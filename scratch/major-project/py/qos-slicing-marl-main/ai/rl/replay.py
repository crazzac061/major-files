import numpy as np, torch
from ai.spec import N_GNB, OBS_DIM, ACT_DIM

class Replay:
    def __init__(self, cap=100_000):
        self.cap, self.i, self.size = cap, 0, 0
        self.o  = np.zeros((cap, N_GNB, OBS_DIM), np.float32)
        self.a  = np.zeros((cap, N_GNB, ACT_DIM), np.float32)   # stored in [-1,1]
        self.r  = np.zeros(cap, np.float32)                     # team reward
        self.o2 = np.zeros((cap, N_GNB, OBS_DIM), np.float32)

    def add(self, o, a, r, o2):
        j = self.i
        self.o[j], self.a[j], self.r[j], self.o2[j] = o, a, r, o2
        self.i = (j + 1) % self.cap; self.size = min(self.size + 1, self.cap)

    def sample(self, B, norm, dev="cpu"):
        idx = np.random.randint(0, self.size, B)
        t = lambda x: torch.as_tensor(x, device=dev)
        return t(norm(self.o[idx])), t(self.a[idx]), t(self.r[idx]), t(norm(self.o2[idx]))

    def save(self, p):
        n = self.size
        np.savez_compressed(p, o=self.o[:n], a=self.a[:n], r=self.r[:n], o2=self.o2[:n])
    def load(self, p):
        d = np.load(p); n = len(d["r"])
        self.o[:n], self.a[:n], self.r[:n], self.o2[:n] = d["o"], d["a"], d["r"], d["o2"]
        self.i, self.size = n % self.cap, n