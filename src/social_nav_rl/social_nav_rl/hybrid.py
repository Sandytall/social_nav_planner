"""Hybrid controller: blend the RL candidate command with a classical/safe command.

Experimentally tests whether learning improves specific behaviours while a classical component
preserves safety/interpretability. When the predicted encounter is risky (low TTC) it hands
control to the classical command; otherwise it blends by `alpha`. The result still passes the
safety supervisor downstream. Pure and unit-testable.
"""
from dataclasses import dataclass


@dataclass
class HybridConfig:
    alpha: float = 0.5       # RL weight in the blend (0 = all classical, 1 = all RL)
    risk_ttc: float = 2.0    # below this predicted TTC, defer entirely to the classical command


class HybridController:
    def __init__(self, cfg: HybridConfig = None):
        self.cfg = cfg or HybridConfig()

    def combine(self, rl_vw, classical_vw, min_ttc: float = 1e3):
        """Return ((v, w), alpha_used). alpha drops to 0 (classical) under imminent risk."""
        alpha = 0.0 if min_ttc < self.cfg.risk_ttc else self.cfg.alpha
        v = alpha * rl_vw[0] + (1.0 - alpha) * classical_vw[0]
        w = alpha * rl_vw[1] + (1.0 - alpha) * classical_vw[1]
        return (float(v), float(w)), alpha
