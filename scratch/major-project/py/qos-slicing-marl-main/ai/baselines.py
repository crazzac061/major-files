import numpy as np
from ai.spec import N_GNB, ACT_DIM

def _pack(w, p=1.0, beam=0.5):
    a = np.zeros((N_GNB, ACT_DIM), np.float32)
    a[:, :3] = w; a[:, 3] = p; a[:, 4:] = beam
    return a

def static_action(obs):    return _pack(np.ones((N_GNB, 3)) / 3)
def random_action(obs):    return np.random.rand(N_GNB, ACT_DIM).astype(np.float32)

def heuristic_action(obs):
    """Stand-in for the minor-project BWP manager: more weight to slices missing their SLA.
    Replace with the real heuristic's rule once your friend can log it (same inputs: KPIs)."""
    k = obs[:, :12].reshape(N_GNB, 3, 4)
    deficit = np.maximum(0, 1 - k[..., 0]) + np.maximum(0, k[..., 1] - 1) + 0.2
    return _pack(deficit / deficit.sum(1, keepdims=True))