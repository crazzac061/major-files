from collections import deque
import numpy as np
from ai.spec import *
from ai.reward import reward

class SliceEnv:
    """3 agents stepped together. obs (3, OBS_DIM); action01 (3, ACT_DIM) in [0,1]."""
    def __init__(self, backend, predictor=None, steps=EPISODE_STEPS):
        self.backend, self.predictor, self.max_steps = backend, predictor, steps
        self.w_a1 = np.ones(3) / 3

    def set_w_a1(self, w): self.w_a1 = np.asarray(w, dtype=np.float32)

    def reset(self, seed=None):
        raw, info = self.backend.reset(seed)
        self.t = 0
        k6 = raw["kpi"][:, TPUT_DELAY_IDX]
        self.hist = deque([k6] * 10, maxlen=10)
        self.last_k6, self.raw = k6, raw
        return self._obs(raw), info

    def step(self, action01):
        raw, info = self.backend.step(np.clip(action01, 0, 1))
        self.t += 1
        k6 = raw["kpi"][:, TPUT_DELAY_IDX]
        self.hist.append(k6); self.last_k6, self.raw = k6, raw
        r, c = reward(raw["kpi"], self.w_a1, raw["p_energy"])
        info.update(c=c, kpi=raw["kpi"], ho=raw["ho"])
        return self._obs(raw), r, False, self.t >= self.max_steps, info

    def _obs(self, raw):
        k = raw["kpi"]
        if self.predictor is not None and self.predictor.ready:
            khat = self.predictor.predict(np.stack(self.hist, 1))   # (3,10,6)
        else:
            khat = k[:, TPUT_DELAY_IDX]                              # naive: repeat last
        return np.concatenate([k, khat, np.tile(self.w_a1, (N_GNB, 1)),
                               raw["ris_gain"], raw["p_energy"][:, None], raw["ho"][:, None]],
                              axis=1).astype(np.float32)