import numpy as np

PRB_LIMIT, DROP_TOL = 0.95, 1e-3

def slice_scores(kpi):
    """kpi: (..., 12) normalised -> (..., 3) compliance c_s in [0,1]."""
    k = kpi.reshape(*kpi.shape[:-1], 3, 4)
    tput, delay, prb, drop = k[..., 0], k[..., 1], k[..., 2], k[..., 3]
    s_t = np.minimum(1.0, tput)
    s_d = np.where(delay <= 1, 1.0, np.maximum(0.0, 1 - (delay - 1)))
    s_p = np.where(prb <= PRB_LIMIT, 1.0, np.maximum(0.0, 1 - (prb - PRB_LIMIT) / 0.05))
    s_x = np.where(drop <= DROP_TOL, 1.0, np.maximum(0.0, 1 - drop * 10))
    return (s_t + s_d + s_p + s_x) / 4.0

def reward(kpi, w_a1, p_energy, lam_e=0.1):
    """kpi (3,12), w_a1 (3,) summing to 1, p_energy (3,) -> per-agent reward (3,), c (3,3)."""
    c = slice_scores(kpi)
    r = (3.0 * w_a1[None, :] * np.log1p(c)).sum(1) - lam_e * p_energy
    return r.astype(np.float32), c

def sla_met(c):
    """True when every slice is fully compliant."""
    return bool(np.all(c >= 0.999))