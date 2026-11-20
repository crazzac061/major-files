import numpy as np, torch
from ai.spec import *
from ai.rl.sac_ctde import SACCTDE

def test_update_runs():
    ag = SACCTDE(); B = 64
    batch = (torch.randn(B, N_GNB, OBS_DIM), torch.rand(B, N_GNB, ACT_DIM) * 2 - 1,
             torch.randn(B), torch.randn(B, N_GNB, OBS_DIM))
    for _ in range(5): s = ag.update(batch)
    assert all(np.isfinite(v) for v in s.values())

def test_decentralised_execution():
    ag = SACCTDE(); o = np.random.randn(N_GNB, OBS_DIM).astype(np.float32)
    a_full = ag.act(o, det=True)
    o2 = o.copy(); o2[1:] = 0                       # change the OTHER agents' observations
    assert np.allclose(a_full[0], ag.act(o2, det=True)[0], atol=1e-5)