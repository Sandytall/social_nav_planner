#!/usr/bin/env python3
"""SocialNav simulation control panel.

A small local web app to configure and launch the SocialNav simulation without remembering
env vars and launch arguments: pick the environment, scenario, people, robots, sensor noise,
view mode and goal, then Launch / Stop. It shells out to the existing run scripts
(run_social_demo.sh / run_factory_demo.sh), which already handle the ROS/Gazebo environment,
the NVIDIA offload, teardown and goal send - this panel only builds their env and runs them.

    python3 scripts/sim_control_panel.py        # then open http://127.0.0.1:8080

Stdlib only (no Flask). Binds to localhost. Every input is validated against a fixed set of
options before it reaches a subprocess, so the page cannot inject a command.
"""
import json
import os
import signal
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(WS, "scripts")
LOG_PATH = "/tmp/social_nav_sim_panel.log"

# Options mirror the real launch scenarios. Keep in sync with urban_demo.launch.py /
# factory_demo.launch.py SCENARIOS.
URBAN_SCENARIOS = [
    "empty", "single_crossing", "crossing", "walking", "same_direction", "head_on",
    "turning", "sudden_stop", "blocker", "two_crossing", "group", "group_merge",
    "crowded", "high_density", "mixed",
]
FACTORY_SCENARIOS = ["factory_empty", "factory_workers", "factory_crossing"]
SENSOR_PROFILES = ["clean", "realistic", "stress"]
VIEWS = {  # view -> (GUI, RVIZ)
    "both": ("true", "true"),
    "gazebo": ("true", "false"),
    "rviz": ("false", "true"),
    "headless": ("false", "false"),
}

# The one running simulation this panel manages (its run-script process group).
_state = {"proc": None, "config": None}


def _running():
    """True if a sim is up: our launched process is alive, or a gzserver lingers."""
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
    """Validate a config dict and return (script_path, env_overrides, resolved_config)."""
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
    people = _clampi(cfg.get("people"), 0, 40, -1)      # -1 => keep the scenario default
    robots = _clampi(cfg.get("robots"), 1, 5, 1)
    goal_x = _clampf(cfg.get("goal_x"), -10.0, 10.0, 9.0 if env == "urban" else 8.5)
    goal_y = _clampf(cfg.get("goal_y"), -10.0, 10.0, 0.0)

    overrides = {
        "SCENARIO": scenario, "GUI": gui, "RVIZ": rviz, "SENSOR_PROFILE": sensor,
        "GOAL_X": str(goal_x), "GOAL_Y": str(goal_y),
    }
    if env == "urban":  # only the urban launch accepts these
        overrides["NUM_ROBOTS"] = str(robots)
        overrides["NUM_HUMANS"] = str(people)

    resolved = {"environment": env, "scenario": scenario, "sensor": sensor, "view": view,
                "people": people, "robots": robots, "goal_x": goal_x, "goal_y": goal_y}
    return script, overrides, resolved


def _launch(cfg):
    if _running():
        return {"ok": False, "error": "A simulation is already running. Stop it first."}
    script, overrides, resolved = _build(cfg)
    if not os.path.isfile(script):
        return {"ok": False, "error": f"Run script not found: {script} (build the workspace?)"}
    env = dict(os.environ)
    env.update(overrides)
    logf = open(LOG_PATH, "w")
    # Own process group so Stop can signal the whole launch tree.
    proc = subprocess.Popen(["bash", script], cwd=WS, env=env,
                            stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
    _state["proc"] = proc
    _state["config"] = resolved
    return {"ok": True, "config": resolved, "log": LOG_PATH}


def _stop():
    proc = _state["proc"]
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
    # stop.sh SIGKILLs any lingering Gazebo/Nav2 (Classic ignores polite signals).
    subprocess.run(["bash", os.path.join(SCRIPTS, "stop.sh")],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _state["proc"] = None
    return {"ok": True}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):  # quiet console
        pass

    def _send(self, code, body, ctype="application/json"):
        payload = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/status":
            self._send(200, json.dumps({"running": _running(), "config": _state["config"]}))
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
    --bg:#11161c; --panel:#1a222c; --panel2:#161d25; --line:#26303b; --fg:#e6edf3;
    --muted:#9aa7b4; --accent:#3fa15b; --accent-fg:#eafff0; --danger:#c8503a; --focus:#5b8cff;
    color-scheme: dark;
  }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:15px/1.5 system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  header { padding:20px 24px; border-bottom:1px solid var(--line); }
  header h1 { margin:0; font-size:18px; font-weight:600; letter-spacing:.01em; }
  header p { margin:4px 0 0; color:var(--muted); font-size:13px; }
  main { display:grid; grid-template-columns:minmax(0,1.4fr) minmax(0,1fr); gap:20px;
         max-width:920px; margin:24px auto; padding:0 16px; }
  @media (max-width:720px){ main{ grid-template-columns:1fr; } }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:12px; padding:18px; }
  .card h2 { margin:0 0 14px; font-size:12px; text-transform:uppercase; letter-spacing:.06em;
             color:var(--muted); font-weight:600; }
  .field { margin-bottom:16px; }
  .field > label { display:block; font-size:13px; margin-bottom:6px; }
  .hint { color:var(--muted); font-size:12px; margin-top:5px; }
  select, input[type=number] { width:100%; background:var(--panel2); color:var(--fg);
    border:1px solid var(--line); border-radius:8px; padding:9px 10px; font-size:14px; }
  select:focus, input:focus { outline:2px solid var(--focus); outline-offset:0; }
  .seg { display:flex; gap:6px; flex-wrap:wrap; }
  .seg label { flex:1 1 auto; text-align:center; padding:8px 10px; border:1px solid var(--line);
    border-radius:8px; background:var(--panel2); cursor:pointer; font-size:13px; white-space:nowrap; }
  .seg input { position:absolute; opacity:0; pointer-events:none; }
  .seg label:has(input:checked){ border-color:var(--accent); background:#1e3327; color:var(--accent-fg); }
  .row { display:flex; gap:12px; } .row > * { flex:1; }
  .actions { display:flex; gap:10px; margin-top:4px; }
  button { border:0; border-radius:9px; padding:11px 16px; font-size:14px; font-weight:600;
    cursor:pointer; }
  button.primary { background:var(--accent); color:var(--accent-fg); flex:2; }
  button.ghost { background:transparent; color:var(--fg); border:1px solid var(--line); flex:1; }
  button:disabled { opacity:.45; cursor:not-allowed; }
  .status { display:flex; align-items:center; gap:9px; font-size:14px; margin-bottom:14px; }
  .dot { width:9px; height:9px; border-radius:50%; background:var(--muted); flex:none; }
  .dot.run { background:var(--accent); box-shadow:0 0 0 4px #3fa15b33; }
  .dot.err { background:var(--danger); }
  dl { margin:0; display:grid; grid-template-columns:auto 1fr; gap:6px 14px; font-size:13px; }
  dl dt { color:var(--muted); } dl dd { margin:0; }
  .banner { border-radius:8px; padding:10px 12px; font-size:13px; margin-bottom:14px; display:none; }
  .banner.err { display:block; background:#331c18; border:1px solid var(--danger); color:#f2c4b8; }
  .empty { color:var(--muted); font-size:13px; }
  code { background:var(--panel2); padding:1px 5px; border-radius:5px; font-size:12px; }
</style>
</head>
<body>
<header>
  <h1>SocialNav — Simulation Control</h1>
  <p>Configure a socially-aware navigation run and launch it. The server drives the same
     scripts you'd run by hand.</p>
</header>
<main>
  <section class="card" aria-label="Configuration">
    <h2>Scenario</h2>
    <div class="field">
      <label>Environment</label>
      <div class="seg" id="env">
        <label><input type="radio" name="environment" value="urban" checked> Urban street</label>
        <label><input type="radio" name="environment" value="factory"> Factory floor</label>
      </div>
    </div>
    <div class="field">
      <label for="scenario">Scenario</label>
      <select id="scenario"></select>
      <div class="hint" id="scenarioHint"></div>
    </div>
    <div class="row">
      <div class="field">
        <label for="people">People</label>
        <input type="number" id="people" min="0" max="40" step="1" value="-1">
        <div class="hint">−1 = scenario default</div>
      </div>
      <div class="field">
        <label for="robots">Robots</label>
        <input type="number" id="robots" min="1" max="5" step="1" value="1">
        <div class="hint">1 primary + extra AMRs</div>
      </div>
    </div>
    <div class="field">
      <label>Sensor noise</label>
      <div class="seg" id="sensor">
        <label><input type="radio" name="sensor" value="clean"> Clean</label>
        <label><input type="radio" name="sensor" value="realistic" checked> Realistic</label>
        <label><input type="radio" name="sensor" value="stress"> Stress</label>
      </div>
    </div>
    <div class="field">
      <label>View</label>
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
    <div class="banner err" id="banner"></div>
    <div class="status"><span class="dot" id="dot"></span><span id="statusText">Idle</span></div>
    <div id="detail"><p class="empty">No simulation running. Configure a scenario and press
      Launch.</p></div>
  </section>
</main>
<script>
  const SCENARIOS = {
    urban: %URBAN%,
    factory: %FACTORY%,
  };
  const $ = (id) => document.getElementById(id);
  const scenarioSel = $("scenario");

  function currentEnv(){ return document.querySelector('input[name=environment]:checked').value; }
  function radio(name){ return document.querySelector('input[name='+name+']:checked').value; }

  function fillScenarios(){
    const env = currentEnv();
    scenarioSel.innerHTML = "";
    SCENARIOS[env].forEach((s,i) => {
      const o = document.createElement("option");
      o.value = s; o.textContent = s.replace(/_/g," ");
      if (i===0 || s==="crossing") o.selected = (s==="crossing");
      scenarioSel.appendChild(o);
    });
    const factory = env === "factory";
    $("people").disabled = factory; $("robots").disabled = factory;
    $("scenarioHint").textContent = factory
      ? "Factory scenarios set their own worker count."
      : "People / Robots below override the scenario.";
  }
  document.querySelectorAll('input[name=environment]').forEach(
    el => el.addEventListener("change", fillScenarios));
  fillScenarios();

  function config(){
    return {
      environment: currentEnv(), scenario: scenarioSel.value,
      people: $("people").value, robots: $("robots").value,
      sensor: radio("sensor"), view: radio("view"),
      goal_x: $("gx").value, goal_y: $("gy").value,
    };
  }

  function render(st){
    const running = st.running;
    $("launch").disabled = running;
    $("stop").disabled = !running;
    $("dot").className = "dot" + (running ? " run" : "");
    $("statusText").textContent = running ? "Running" : "Idle";
    const c = st.config;
    if (running && c){
      $("detail").innerHTML =
        '<dl><dt>Environment</dt><dd>'+c.environment+'</dd>'+
        '<dt>Scenario</dt><dd>'+c.scenario+'</dd>'+
        '<dt>People</dt><dd>'+(c.people<0?'scenario default':c.people)+'</dd>'+
        '<dt>Robots</dt><dd>'+c.robots+'</dd>'+
        '<dt>Sensor</dt><dd>'+c.sensor+'</dd>'+
        '<dt>View</dt><dd>'+c.view+'</dd>'+
        '<dt>Goal</dt><dd>('+c.goal_x+', '+c.goal_y+')</dd></dl>'+
        '<p class="hint">Log: <code>/tmp/social_nav_sim_panel.log</code></p>';
    } else if (!running) {
      $("detail").innerHTML = '<p class="empty">No simulation running. Configure a scenario '+
        'and press Launch.</p>';
    }
  }
  function showError(msg){
    const b = $("banner"); b.textContent = msg; b.classList.add("err");
    $("dot").className = "dot err";
  }
  function clearError(){ const b=$("banner"); b.textContent=""; b.classList.remove("err"); }

  async function poll(){
    try { render(await (await fetch("/status")).json()); } catch(e){}
  }
  async function post(path, body){
    const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"},
      body: JSON.stringify(body||{})});
    return r.json();
  }
  $("launch").addEventListener("click", async () => {
    clearError();
    $("launch").disabled = true; $("statusText").textContent = "Launching…";
    const res = await post("/launch", config());
    if (!res.ok){ showError(res.error||"Launch failed."); $("launch").disabled = false; }
    poll();
  });
  $("stop").addEventListener("click", async () => {
    clearError(); $("statusText").textContent = "Stopping…";
    await post("/stop", {}); poll();
  });
  poll(); setInterval(poll, 2500);
</script>
</body>
</html>
"""
PAGE = PAGE.replace("%URBAN%", json.dumps(URBAN_SCENARIOS)) \
           .replace("%FACTORY%", json.dumps(FACTORY_SCENARIOS))


def main():
    port = int(os.environ.get("PORT", "8080"))
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    url = f"http://127.0.0.1:{port}"
    print(f"SocialNav control panel: {url}  (Ctrl-C to quit)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down; stopping any running simulation...")
        _stop()
        server.shutdown()


if __name__ == "__main__":
    sys.exit(main())
