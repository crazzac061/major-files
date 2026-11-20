import numpy as np
from ai.reward import sla_met

def run_episode(env, policy, seed):
    """policy: obs (3,25) -> action01 (3,6). Returns per-episode metrics."""
    obs, _ = env.reset(seed=seed); env.set_w_a1(np.ones(3) / 3)
    sla, comp, ret, power, ho_steps = [], [], 0.0, [], []
    while True:
        obs, r, _, trunc, info = env.step(policy(obs))
        sla.append(sla_met(info["c"])); comp.append(info["c"].mean(0))   # per-slice mean over gNBs
        ret += r.sum(); power.append(info["power"].mean()); ho_steps.append(info["ho"].any())
        if trunc: break
    comp = np.array(comp)
    return dict(sla_rate=float(np.mean(sla)), ret=float(ret),
                c_urllc=comp[:, 0].mean(), c_embb=comp[:, 1].mean(), c_mmtc=comp[:, 2].mean(),
                jain_inter=float(comp.mean(0).sum() ** 2 / (3 * (comp.mean(0) ** 2).sum() + 1e-9)),
                mean_power=float(np.mean(power)))

def sac_policy(agent, norm):
    return lambda obs: (agent.act(norm(obs), det=True) + 1) / 2