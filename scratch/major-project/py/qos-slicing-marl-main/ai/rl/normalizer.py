import numpy as np

class RunningNorm:
    """Fit once on warm-start data, then freeze (Suggested in the plan)."""
    def __init__(self, dim): self.mean, self.std = np.zeros(dim, np.float32), np.ones(dim, np.float32)
    def fit(self, x):
        self.mean = x.mean(0).astype(np.float32)
        s = x.std(0); s[s < 1e-3] = 1.0          # constant features (w_A1, zero RIS) stay as-is
        self.std = s.astype(np.float32)
    def __call__(self, x): return np.clip((x - self.mean) / self.std, -5, 5).astype(np.float32)
    def save(self, p): np.savez(p, mean=self.mean, std=self.std)
    def load(self, p): d = np.load(p); self.mean, self.std = d["mean"], d["std"]