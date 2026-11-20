import numpy as np
from ai.spec import *
from ai.envs.mock_backend import MockBackend
from ai.envs.slice_env import SliceEnv

def rollout(seed):
    env = SliceEnv(MockBackend()); obs, _ = env.reset(seed=seed)
    out, rng = [obs], np.random.default_rng(0)
    for _ in range(50):
        obs, r, _, _, _ = env.step(rng.random((N_GNB, ACT_DIM))); out.append(obs)
    return np.stack(out)

def test_shapes_and_finite():
    o = rollout(1)
    assert o.shape == (51, N_GNB, OBS_DIM) and np.isfinite(o).all()

def test_determinism():
    assert np.array_equal(rollout(7), rollout(7))

def test_weights_change_kpis():
    env = SliceEnv(MockBackend()); env.reset(seed=3)
    a = np.zeros((N_GNB, ACT_DIM)); a[:, 3] = 1; a[:, 4:] = .5
    a[:, :3] = [.8, .1, .1]; o1 = env.step(a)[0]
    env.reset(seed=3); a[:, :3] = [.1, .8, .1]; o2 = env.step(a)[0]
    assert not np.allclose(o1[:, :12], o2[:, :12])