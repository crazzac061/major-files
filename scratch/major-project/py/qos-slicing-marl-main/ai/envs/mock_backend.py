import numpy as np
from ai.spec import N_GNB, R, W_FLOOR

BASE_LOAD = np.array([0.35, 0.45, 0.15])   # uRLLC, eMBB, mMTC demand vs capacity 1.6

class MockBackend:
    """Stands in for ns-3. Same interface as Ns3Backend: reset(seed), step(action01)."""
    def __init__(self, adversarial=False, cyclic=False):
        self.adversarial, self.cyclic = adversarial, cyclic
        self.rng = np.random.default_rng(0); self.t = 0

    def reset(self, seed=None):
        self.rng = np.random.default_rng(seed)
        self.t = 0
        self.opt_beam = self.rng.integers(4, 12, size=(N_GNB, R))
        self.skew = self.rng.uniform(0.8, 1.2, size=(N_GNB, 3))
        a = np.zeros((N_GNB, 3 + 1 + R)); a[:, :3] = 1 / 3; a[:, 3] = 1.0; a[:, 4:] = 0.5
        return self._simulate(a, 0.0), {"t_s": 0.0}

    def step(self, action01):
        a = np.nan_to_num(np.clip(action01, 0, 1))
        self.t += 1
        raw = self._simulate(a, self.t * 0.1)
        return raw, {"t_s": self.t * 0.1, "power": raw["p_energy"].copy()}

    def _phase(self, t_s):
        if self.cyclic: t_s = t_s % 50
        m = np.ones((N_GNB, 3))
        if 15 <= t_s < 30:   m[:, 0] = 2.0; m[:, 1] = 0.4      # uRLLC burst, eMBB reduced
        elif 30 <= t_s < 45: m[:, 1] = 2.0                      # eMBB surge
        elif t_s >= 45:      m[:, 2] = 10.0                     # mMTC flood
        if self.adversarial and t_s >= 25: m[:, 2] *= 50.0      # misbehaving mMTC
        return m

    def _simulate(self, a, t_s):
        w = np.maximum(a[:, :3], W_FLOOR); w = w / w.sum(1, keepdims=True)
        p = a[:, 3]
        beams = np.minimum((a[:, 4:] * 16).astype(int), 15)
        load = BASE_LOAD * self.skew * self._phase(t_s) * self.rng.normal(1, 0.05, (N_GNB, 3))
        ris_gain = 6.0 * np.exp(-((beams - self.opt_beam) / 3.0) ** 2)       # dB-like
        cap = 1.6 * w * ((0.5 + 0.5 * p) * (1 + 0.03 * ris_gain.sum(1)))[:, None]
        served = np.minimum(load, cap)
        rho = load / cap
        tput = served / load
        delay = np.clip(0.4 + 0.6 * rho ** 2, 0, 5)
        prb = np.clip(rho, 0, 1)
        drop = np.clip(1 - cap / load, 0, 1)
        kpi = np.stack([tput, delay, prb, drop], -1).reshape(N_GNB, 12)
        return {"kpi": kpi.astype(np.float32),
                "ris_gain": ris_gain.astype(np.float32),
                "p_energy": p.astype(np.float32),
                "ho": (self.rng.random(N_GNB) < 0.01).astype(np.float32)}