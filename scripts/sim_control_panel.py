#!/usr/bin/env python3
"""SocialNav simulation control panel.

A local web app to run the SocialNav simulation and its benchmarks without remembering env
vars and launch arguments. Two tabs:

  * Launch    - pick environment, scenario, people, robots, sensor noise, view and goal,
                then Launch / Stop a live Gazebo demo.
  * Benchmarks- run a headless benchmark sweep (optionally comparing planners) and open the
                generated full HTML report in the browser.

It shells out to the existing run scripts and the social-nav-benchmark CLI, which already
handle the ROS/Gazebo environment, the NVIDIA offload, teardown and goal send.

    python3 scripts/sim_control_panel.py        # then open http://127.0.0.1:8080

Stdlib only (no Flask). Binds to localhost. Every input is validated against a fixed set of
options before it reaches a subprocess, so the page cannot inject a command. Only one Gazebo
activity (a demo or a benchmark) runs at a time.
"""
import json
import os
import signal
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(WS, "scripts")
RESULTS_ROOT = os.path.expanduser("~/social_nav_results")
LOG_PATH = "/tmp/social_nav_sim_panel.log"

# Options mirror the real launch scenarios and installed controllers.
URBAN_SCENARIOS = [
    "empty", "single_crossing", "crossing", "walking", "same_direction", "head_on",
    "turning", "sudden_stop", "blocker", "two_crossing", "group", "group_merge",
    "crowded", "high_density", "mixed",
]
FACTORY_SCENARIOS = ["factory_empty", "factory_workers", "factory_crossing"]
BENCH_SCENARIOS = ["all"] + URBAN_SCENARIOS
PLANNERS = ["social_nav", "rpp", "dwb", "mppi"]
SENSOR_PROFILES = ["clean", "realistic", "stress"]
VIEWS = {"both": ("true", "true"), "gazebo": ("true", "false"),
         "rviz": ("false", "true"), "headless": ("false", "false")}

# The one running Gazebo activity this panel manages (a demo or a benchmark).
_state = {"proc": None, "kind": None, "config": None}


def _running():
    proc = _state["proc"]
    if proc is not None and proc.poll() is None:
        return True
    return subprocess.run(["pgrep", "-x", "gzserver"],
                          stdout=subprocess.DEVNULL).returncode == 0


def _clampi(value, lo, hi, default):
    try:
        return max(lo, min(hi, int(value)))
    except (TypeError, ValueError):
        return default


def _clampf(value, lo, hi, default):
    try:
        return max(lo, min(hi, float(value)))
    except (TypeError, ValueError):
        return default


def _build(cfg):
    """Validate a demo config and return (script_path, env_overrides, resolved_config)."""
    env = cfg.get("environment")
    if env == "factory":
        script = os.path.join(SCRIPTS, "run_factory_demo.sh")
        scenarios = FACTORY_SCENARIOS
    else:
        env = "urban"
        script = os.path.join(SCRIPTS, "run_social_demo.sh")
        scenarios = URBAN_SCENARIOS
    scenario = cfg.get("scenario") if cfg.get("scenario") in scenarios else scenarios[0]
    sensor = cfg.get("sensor") if cfg.get("sensor") in SENSOR_PROFILES else "realistic"
    view = cfg.get("view") if cfg.get("view") in VIEWS else "both"
    gui, rviz = VIEWS[view]
    people = _clampi(cfg.get("people"), 0, 40, -1)
    robots = _clampi(cfg.get("robots"), 1, 5, 1)
    goal_x = _clampf(cfg.get("goal_x"), -10.0, 10.0, 9.0 if env == "urban" else 8.5)
    goal_y = _clampf(cfg.get("goal_y"), -10.0, 10.0, 0.0)
    overrides = {"SCENARIO": scenario, "GUI": gui, "RVIZ": rviz, "SENSOR_PROFILE": sensor,
                 "GOAL_X": str(goal_x), "GOAL_Y": str(goal_y)}
    if env == "urban":
        overrides["NUM_ROBOTS"] = str(robots)
        overrides["NUM_HUMANS"] = str(people)
    resolved = {"kind": "demo", "environment": env, "scenario": scenario, "sensor": sensor,
                "view": view, "people": people, "robots": robots,
                "goal": [goal_x, goal_y]}
    return script, overrides, resolved


def _launch(cfg):
    if _running():
        return {"ok": False, "error": "A simulation or benchmark is already running. Stop it first."}
    script, overrides, resolved = _build(cfg)
    if not os.path.isfile(script):
        return {"ok": False, "error": f"Run script not found: {script} (build the workspace?)"}
    env = dict(os.environ)
    env.update(overrides)
    logf = open(LOG_PATH, "w")
    proc = subprocess.Popen(["bash", script], cwd=WS, env=env,
                            stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
    _state.update(proc=proc, kind="demo", config=resolved)
    return {"ok": True, "config": resolved}


def _benchmark(cfg):
    if _running():
        return {"ok": False, "error": "A simulation or benchmark is already running. Stop it first."}
    scenario = cfg.get("scenario") if cfg.get("scenario") in BENCH_SCENARIOS else "all"
    runs = _clampi(cfg.get("runs"), 1, 50, 5)
    planners = [p for p in (cfg.get("planners") or []) if p in PLANNERS]
    # All tokens are allowlisted above, so this command string carries no user text.
    cmd = (f"source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && "
           f"social-nav-benchmark --scenario {scenario} --runs {runs}")
    if planners:
        cmd += f" --planner {','.join(planners)}"
    logf = open(LOG_PATH, "w")
    proc = subprocess.Popen(["bash", "-c", cmd], cwd=WS, env=dict(os.environ),
                            stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
    resolved = {"kind": "benchmark", "scenario": scenario, "runs": runs,
                "planners": planners or ["(default)"]}
    _state.update(proc=proc, kind="benchmark", config=resolved)
    return {"ok": True, "config": resolved}


def _stop():
    proc = _state["proc"]
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
    subprocess.run(["bash", os.path.join(SCRIPTS, "stop.sh")],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _state.update(proc=None, kind=None)
    return {"ok": True}


def _list_reports():
    """Result runs that carry a report.html, newest first."""
    out = []
    if os.path.isdir(RESULTS_ROOT):
        for name in os.listdir(RESULTS_ROOT):
            report = os.path.join(RESULTS_ROOT, name, "report.html")
            if os.path.isfile(report):
                out.append({"name": name, "mtime": int(os.path.getmtime(report))})
    out.sort(key=lambda r: r["mtime"], reverse=True)
    return out


def _read_report(name):
    """Return the bytes of RESULTS_ROOT/<name>/report.html, or None if the name is unsafe."""
    if not name or "/" in name or "\\" in name or name.startswith("."):
        return None
    path = os.path.realpath(os.path.join(RESULTS_ROOT, name, "report.html"))
    if not path.startswith(os.path.realpath(RESULTS_ROOT) + os.sep):
        return None
    try:
        with open(path, "rb") as f:
            return f.read()
    except OSError:
        return None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def _send(self, code, body, ctype="application/json"):
        payload = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif parsed.path == "/status":
            self._send(200, json.dumps({"running": _running(), "kind": _state["kind"],
                                        "config": _state["config"]}))
        elif parsed.path == "/reports":
            self._send(200, json.dumps({"reports": _list_reports(),
                                        "root": RESULTS_ROOT}))
        elif parsed.path == "/report":
            name = (parse_qs(parsed.query).get("name") or [""])[0]
            body = _read_report(name)
            if body is None:
                self._send(404, "report not found", "text/plain")
            else:
                self._send(200, body, "text/html; charset=utf-8")
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            cfg = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            self._send(400, json.dumps({"ok": False, "error": "bad JSON"}))
            return
        if self.path == "/launch":
            self._send(200, json.dumps(_launch(cfg)))
        elif self.path == "/benchmark":
            self._send(200, json.dumps(_benchmark(cfg)))
        elif self.path == "/stop":
            self._send(200, json.dumps(_stop()))
        else:
            self._send(404, json.dumps({"error": "not found"}))


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SocialNav Control</title>
<style>
  :root {
    --bg:#0f141a; --panel:#171f28; --panel2:#121820; --line:#26313d; --fg:#e8eef4;
    --muted:#93a1b1; --accent:#41a35c; --accent-fg:#eafff0; --danger:#c8503a;
    --focus:#5b8cff; --amber:#d9a441; color-scheme:dark;
  }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
    font:15px/1.55 system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  header { display:flex; align-items:center; gap:14px; padding:16px 24px;
    border-bottom:1px solid var(--line); background:linear-gradient(180deg,#141b23,#0f141a); }
  .logo { width:30px; height:30px; border-radius:8px; flex:none;
    background:linear-gradient(135deg,var(--accent),#2c7fbf); }
  header h1 { margin:0; font-size:16px; font-weight:650; letter-spacing:.01em; }
  header p { margin:1px 0 0; color:var(--muted); font-size:12.5px; }
  .pill { margin-left:auto; display:flex; align-items:center; gap:8px; font-size:13px;
    background:var(--panel); border:1px solid var(--line); border-radius:999px; padding:6px 13px; }
  .dot { width:9px; height:9px; border-radius:50%; background:var(--muted); flex:none; }
  .dot.run { background:var(--accent); box-shadow:0 0 0 4px #41a35c33; }
  .dot.err { background:var(--danger); }
  .tabs { display:flex; gap:4px; padding:14px 24px 0; border-bottom:1px solid var(--line); }
  .tab { padding:9px 16px; border:1px solid transparent; border-bottom:none; cursor:pointer;
    color:var(--muted); font-size:14px; border-radius:9px 9px 0 0; }
  .tab[aria-selected=true] { color:var(--fg); background:var(--panel); border-color:var(--line); }
  main { max-width:960px; margin:22px auto; padding:0 16px; }
  .grid { display:grid; grid-template-columns:minmax(0,1.4fr) minmax(0,1fr); gap:20px; }
  @media (max-width:760px){ .grid{ grid-template-columns:1fr; } }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:13px; padding:18px; }
  .card h2 { margin:0 0 14px; font-size:11.5px; text-transform:uppercase; letter-spacing:.07em;
    color:var(--muted); font-weight:650; }
  .field { margin-bottom:15px; }
  .field > label { display:block; font-size:13px; margin-bottom:6px; }
  .hint { color:var(--muted); font-size:12px; margin-top:5px; }
  select, input[type=number] { width:100%; background:var(--panel2); color:var(--fg);
    border:1px solid var(--line); border-radius:8px; padding:9px 10px; font-size:14px; }
  select:focus, input:focus { outline:2px solid var(--focus); outline-offset:0; }
  .seg { display:flex; gap:6px; flex-wrap:wrap; }
  .seg label { flex:1 1 auto; text-align:center; padding:8px 10px; border:1px solid var(--line);
    border-radius:8px; background:var(--panel2); cursor:pointer; font-size:13px; white-space:nowrap; }
  .seg input { position:absolute; opacity:0; pointer-events:none; }
  .seg label:has(input:checked){ border-color:var(--accent); background:#193026; color:var(--accent-fg); }
  .chips { display:flex; gap:8px; flex-wrap:wrap; }
  .chips label { display:flex; align-items:center; gap:7px; padding:7px 12px; border:1px solid var(--line);
    border-radius:999px; background:var(--panel2); cursor:pointer; font-size:13px; }
  .chips label:has(input:checked){ border-color:var(--accent); color:var(--accent-fg); }
  .row { display:flex; gap:12px; } .row > * { flex:1; }
  .actions { display:flex; gap:10px; margin-top:4px; }
  button { border:0; border-radius:9px; padding:11px 16px; font-size:14px; font-weight:650; cursor:pointer; }
  button.primary { background:var(--accent); color:var(--accent-fg); flex:2; }
  button.ghost { background:transparent; color:var(--fg); border:1px solid var(--line); flex:1; }
  button:disabled { opacity:.45; cursor:not-allowed; }
  dl { margin:0; display:grid; grid-template-columns:auto 1fr; gap:6px 14px; font-size:13px; }
  dl dt { color:var(--muted); } dl dd { margin:0; }
  .banner { border-radius:8px; padding:10px 12px; font-size:13px; margin-bottom:14px; display:none; }
  .banner.show { display:block; background:#331c18; border:1px solid var(--danger); color:#f2c4b8; }
  .empty { color:var(--muted); font-size:13px; }
  .replist { display:flex; flex-direction:column; gap:8px; }
  .rep { display:flex; align-items:center; gap:12px; padding:10px 12px; background:var(--panel2);
    border:1px solid var(--line); border-radius:9px; }
  .rep .name { font-size:14px; } .rep .when { color:var(--muted); font-size:12px; }
  .rep button { margin-left:auto; padding:7px 13px; background:transparent; border:1px solid var(--line);
    color:var(--fg); }
  code { background:var(--panel2); padding:1px 5px; border-radius:5px; font-size:12px; }
  .hidden { display:none; }
</style>
</head>
<body>
<header>
  <div class="logo" aria-hidden="true"></div>
  <div>
    <h1>SocialNav — Simulation Control</h1>
    <p>Configure a socially-aware navigation run, or benchmark planners and read the report.</p>
  </div>
  <div class="pill"><span class="dot" id="dot"></span><span id="pill">Idle</span></div>
</header>

<div class="tabs" role="tablist">
  <div class="tab" role="tab" aria-selected="true" id="tabLaunch"
       onclick="showTab('launch')">Launch</div>
  <div class="tab" role="tab" aria-selected="false" id="tabReports"
       onclick="showTab('reports')">Benchmarks &amp; reports</div>
</div>

<main>
  <div class="banner" id="banner"></div>

  <!-- LAUNCH TAB -->
  <div id="panelLaunch" class="grid">
    <section class="card" aria-label="Configuration">
      <h2>Scenario</h2>
      <div class="field"><label>Environment</label>
        <div class="seg" id="env">
          <label><input type="radio" name="environment" value="urban" checked> Urban street</label>
          <label><input type="radio" name="environment" value="factory"> Factory floor</label>
        </div>
      </div>
      <div class="field"><label for="scenario">Scenario</label>
        <select id="scenario"></select><div class="hint" id="scenarioHint"></div></div>
      <div class="row">
        <div class="field"><label for="people">People</label>
          <input type="number" id="people" min="0" max="40" step="1" value="-1">
          <div class="hint">-1 = scenario default</div></div>
        <div class="field"><label for="robots">Robots</label>
          <input type="number" id="robots" min="1" max="5" step="1" value="1">
          <div class="hint">1 primary + extra AMRs</div></div>
      </div>
      <div class="field"><label>Sensor noise</label>
        <div class="seg" id="sensor">
          <label><input type="radio" name="sensor" value="clean"> Clean</label>
          <label><input type="radio" name="sensor" value="realistic" checked> Realistic</label>
          <label><input type="radio" name="sensor" value="stress"> Stress</label>
        </div>
      </div>
      <div class="field"><label>View</label>
        <div class="seg" id="view">
          <label><input type="radio" name="view" value="both" checked> Gazebo + RViz</label>
          <label><input type="radio" name="view" value="gazebo"> Gazebo</label>
          <label><input type="radio" name="view" value="rviz"> RViz</label>
          <label><input type="radio" name="view" value="headless"> Headless</label>
        </div>
      </div>
      <div class="row">
        <div class="field"><label for="gx">Goal X (m)</label>
          <input type="number" id="gx" step="0.5" value="9"></div>
        <div class="field"><label for="gy">Goal Y (m)</label>
          <input type="number" id="gy" step="0.5" value="0"></div>
      </div>
      <div class="hint" style="margin:-6px 0 14px">Keep the goal within ~10 m of the start
        (rolling costmap window).</div>
      <div class="actions">
        <button class="primary" id="launch">Launch simulation</button>
        <button class="ghost" id="stop" disabled>Stop</button>
      </div>
    </section>
    <section class="card" aria-label="Status">
      <h2>Status</h2>
      <div id="launchDetail"><p class="empty">No simulation running. Configure a scenario and
        press Launch.</p></div>
    </section>
  </div>

  <!-- REPORTS TAB -->
  <div id="panelReports" class="grid hidden">
    <section class="card" aria-label="Run benchmark">
      <h2>Run a benchmark</h2>
      <div class="field"><label for="bscenario">Scenario</label>
        <select id="bscenario"></select></div>
      <div class="field"><label for="bruns">Runs per scenario</label>
        <input type="number" id="bruns" min="1" max="50" step="1" value="5"></div>
      <div class="field"><label>Planners to compare</label>
        <div class="chips" id="planners"></div>
        <div class="hint">social_nav vs. baselines on the same scenarios/seeds.</div></div>
      <div class="actions">
        <button class="primary" id="runBench">Run benchmark</button>
        <button class="ghost" id="stop2" disabled>Stop</button>
      </div>
      <div class="hint" style="margin-top:12px">Headless; a sweep can take many minutes
        (Gazebo per run). The report appears at right when it finishes.</div>
    </section>
    <section class="card" aria-label="Reports">
      <h2>Reports</h2>
      <div id="reports"><p class="empty">Loading…</p></div>
    </section>
  </div>
</main>

<script>
  const SCEN = { urban:%URBAN%, factory:%FACTORY% };
  const BENCH = %BENCH%, PLANNERS = %PLANNERS%;
  const $ = (id) => document.getElementById(id);
  const env = () => document.querySelector('input[name=environment]:checked').value;
  const radio = (n) => document.querySelector('input[name='+n+']:checked').value;

  function showTab(t){
    $("panelLaunch").classList.toggle("hidden", t!=="launch");
    $("panelReports").classList.toggle("hidden", t!=="reports");
    $("tabLaunch").setAttribute("aria-selected", t==="launch");
    $("tabReports").setAttribute("aria-selected", t==="reports");
    if (t==="reports") loadReports();
  }

  function fillScenarios(){
    const e = env();
    $("scenario").innerHTML = "";
    SCEN[e].forEach((s) => {
      const o = document.createElement("option");
      o.value = s; o.textContent = s.replace(/_/g," ");
      if (s==="crossing") o.selected = true;
      $("scenario").appendChild(o);
    });
    const factory = e==="factory";
    $("people").disabled = factory; $("robots").disabled = factory;
    $("scenarioHint").textContent = factory
      ? "Factory scenarios set their own worker count."
      : "People / Robots below override the scenario.";
  }
  document.querySelectorAll('input[name=environment]').forEach(
    el => el.addEventListener("change", fillScenarios));
  fillScenarios();

  // Benchmark controls
  BENCH.forEach((s) => { const o=document.createElement("option");
    o.value=s; o.textContent=s.replace(/_/g," "); $("bscenario").appendChild(o); });
  PLANNERS.forEach((p) => {
    const l=document.createElement("label");
    l.innerHTML='<input type="checkbox" value="'+p+'"'+(p==="social_nav"?" checked":"")+'> '+p;
    $("planners").appendChild(l);
  });

  function demoConfig(){ return {
    environment:env(), scenario:$("scenario").value, people:$("people").value,
    robots:$("robots").value, sensor:radio("sensor"), view:radio("view"),
    goal_x:$("gx").value, goal_y:$("gy").value }; }
  function benchConfig(){ return {
    scenario:$("bscenario").value, runs:$("bruns").value,
    planners:[...document.querySelectorAll('#planners input:checked')].map(c=>c.value) }; }

  function banner(msg){ const b=$("banner"); b.textContent=msg;
    b.classList.toggle("show", !!msg); }

  function render(st){
    const run = st.running, kind = st.kind, c = st.config || {};
    $("dot").className = "dot" + (run ? " run" : "");
    $("pill").textContent = run ? (kind==="benchmark" ? "Benchmark running" : "Simulation running")
                                : "Idle";
    $("launch").disabled = run; $("runBench").disabled = run;
    $("stop").disabled = !run; $("stop2").disabled = !run;
    if (run && kind==="demo"){
      $("launchDetail").innerHTML =
        '<dl><dt>Environment</dt><dd>'+c.environment+'</dd>'+
        '<dt>Scenario</dt><dd>'+c.scenario+'</dd>'+
        '<dt>People</dt><dd>'+(c.people<0?'scenario default':c.people)+'</dd>'+
        '<dt>Robots</dt><dd>'+c.robots+'</dd>'+
        '<dt>Sensor</dt><dd>'+c.sensor+'</dd>'+
        '<dt>View</dt><dd>'+c.view+'</dd>'+
        '<dt>Goal</dt><dd>('+c.goal[0]+', '+c.goal[1]+')</dd></dl>'+
        '<p class="hint">Log: <code>/tmp/social_nav_sim_panel.log</code></p>';
    } else if (!run){
      $("launchDetail").innerHTML = '<p class="empty">No simulation running. Configure a '+
        'scenario and press Launch.</p>';
    } else {
      $("launchDetail").innerHTML = '<p class="empty">A benchmark is running (see the '+
        'Benchmarks tab).</p>';
    }
  }

  async function post(path, body){
    const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"},
      body:JSON.stringify(body||{})}); return r.json(); }
  async function poll(){ try { render(await (await fetch("/status")).json()); } catch(e){} }

  async function loadReports(){
    try {
      const d = await (await fetch("/reports")).json();
      const box = $("reports");
      if (!d.reports.length){ box.innerHTML = '<p class="empty">No reports yet. Run a '+
        'benchmark, or point <code>social-nav-analyze</code> at a results dir. Reports are '+
        'read from <code>'+d.root+'</code>.</p>'; return; }
      box.innerHTML = '<div class="replist">' + d.reports.map(r =>
        '<div class="rep"><div><div class="name">'+r.name+'</div>'+
        '<div class="when">'+new Date(r.mtime*1000).toLocaleString()+'</div></div>'+
        '<button onclick="window.open(\\'/report?name='+encodeURIComponent(r.name)+
        '\\',\\'_blank\\')">Open report</button></div>').join("") + '</div>';
    } catch(e){ $("reports").innerHTML = '<p class="empty">Could not load reports.</p>'; }
  }

  $("launch").addEventListener("click", async () => {
    banner(""); const res = await post("/launch", demoConfig());
    if (!res.ok) banner(res.error||"Launch failed."); poll(); });
  $("runBench").addEventListener("click", async () => {
    banner(""); const res = await post("/benchmark", benchConfig());
    if (!res.ok) banner(res.error||"Could not start benchmark."); poll(); });
  const stopFn = async () => { banner(""); await post("/stop", {}); poll(); };
  $("stop").addEventListener("click", stopFn);
  $("stop2").addEventListener("click", stopFn);

  poll(); setInterval(poll, 2500);
  setInterval(() => { if (!$("panelReports").classList.contains("hidden")) loadReports(); }, 5000);
</script>
</body>
</html>
"""
PAGE = (PAGE.replace("%URBAN%", json.dumps(URBAN_SCENARIOS))
            .replace("%FACTORY%", json.dumps(FACTORY_SCENARIOS))
            .replace("%BENCH%", json.dumps(BENCH_SCENARIOS))
            .replace("%PLANNERS%", json.dumps(PLANNERS)))


def main():
    port = int(os.environ.get("PORT", "8080"))
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"SocialNav control panel: http://127.0.0.1:{port}  (Ctrl-C to quit)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down; stopping any running activity...")
        _stop()
        server.shutdown()


if __name__ == "__main__":
    sys.exit(main())
