import argparse, os, json, numpy as np, torch
from torch.utils.tensorboard import SummaryWriter
from ai.spec import *
from ai.envs.mock_backend import MockBackend
from ai.envs.slice_env import SliceEnv
from ai.rl.replay import Replay
from ai.rl.normalizer import RunningNorm
from ai.rl.sac_ctde import SACCTDE
from ai.rl.lstm_pred import TrafficPredictor
from ai.baselines import heuristic_action, random_action
from ai.evaluate import run_episode, sac_policy
from ai.reward import sla_met
REWARD_SCALE = 1.0

def make_backend(a):
    if a.backend == "mock": return MockBackend()
    from ai.envs.ns3_backend import Ns3Backend
    return Ns3Backend(port=a.port, sim_dir=a.sim_dir)

def collect(env, buf, n, fn, seed0, bonus=0.0):
    ep = 0
    while buf.size < n:
        obs, _ = env.reset(seed=seed0 + ep); ep += 1
        for _ in range(EPISODE_STEPS):
            a01 = fn(obs); obs2, r, _, tr, inf = env.step(a01)
            buf.add(obs, a01 * 2 - 1, r.sum() * REWARD_SCALE + bonus * float(sla_met(inf["c"])), obs2); obs = obs2
            if buf.size >= n or tr: break

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, default=0); p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--warm", choices=["heuristic", "random"], default="heuristic")
    p.add_argument("--start", type=int, default=5000); p.add_argument("--randomize-wa1", action="store_true")
    p.add_argument("--backend", default="mock"); p.add_argument("--port", type=int, default=5555)
    p.add_argument("--lr", type=float, default=3e-4); p.add_argument("--bonus", type=float, default=0.0); p.add_argument("--gamma", type=float, default=0.9); p.add_argument("--sim-dir", default="sim"); p.add_argument("--out", default="runs/exp")
    a = p.parse_args()

    np.random.seed(a.seed); torch.manual_seed(a.seed)
    out = f"{a.out}/seed{a.seed}"; os.makedirs(out, exist_ok=True)
    tb = SummaryWriter(out)
    dev = "cuda" if torch.cuda.is_available() else "cpu"

    pred = TrafficPredictor(); env = SliceEnv(make_backend(a), predictor=pred)
    buf, norm, agent = Replay(), RunningNorm(OBS_DIM), SACCTDE(lr=a.lr, gamma=a.gamma, device=dev)

    fn = (lambda o: heuristic_action(o)) if a.warm == "heuristic" else (lambda o: random_action(o))
    collect(env, buf, a.start, fn, seed0=a.seed * 100_000 + 50_000, bonus=a.bonus)
    buf.save(f"{out}/warm_buffer.npz")
    norm.fit(buf.o[:buf.size].reshape(-1, OBS_DIM)); norm.save(f"{out}/norm.npz")

    gstep, best = 0, -1e9
    for ep in range(a.episodes):
        obs, _ = env.reset(seed=a.seed * 100_000 + ep)         # training seeds
        env.set_w_a1(np.ones(3) / 3); seq, ret = [], 0.0
        for t in range(EPISODE_STEPS):
            if a.randomize_wa1 and t % NONRT_EVERY == 0: env.set_w_a1(np.random.dirichlet([4, 4, 4]))
            act = agent.act(norm(obs))
            obs2, r, _, _, info = env.step((act + 1) / 2)
            buf.add(obs, act, r.sum() * REWARD_SCALE + a.bonus * float(sla_met(info["c"])), obs2); seq.append(env.last_k6.copy())
            obs, ret, gstep = obs2, ret + r.sum(), gstep + 1
            s = agent.update(buf.sample(256, norm, dev))
            if gstep % 100 == 0:
                for k, v in s.items(): tb.add_scalar(f"train/{k}", v, gstep)
        tb.add_scalar("train/episode_return", ret, ep)
        pred.add_episode(np.array(seq)); l = pred.train(steps=100)
        if l is not None: tb.add_scalar("train/lstm_mse", l, ep)

        if (ep + 1) % 5 == 0:                                   # held-out evaluation seeds
            ev = [run_episode(env, sac_policy(agent, norm), 9_000_000 + k) for k in range(5)]
            m = {k: float(np.mean([e[k] for e in ev])) for k in ev[0]}
            for k, v in m.items(): tb.add_scalar(f"eval/{k}", v, ep)
            print(f"ep {ep+1:3d} train_ret {ret:8.1f} eval_ret {m['ret']:8.1f} sla {m['sla_rate']:.3f}")
            agent.save(f"{out}/last.pt"); torch.save(pred.net.state_dict(), f"{out}/lstm_last.pt")
            if m["ret"] > best: best = m["ret"]; agent.save(f"{out}/best.pt"); torch.save(pred.net.state_dict(), f"{out}/lstm_best.pt")
    json.dump({"best_eval_ret": best, "spec": SPEC_VERSION}, open(f"{out}/summary.json", "w"))

if __name__ == "__main__":
    main()