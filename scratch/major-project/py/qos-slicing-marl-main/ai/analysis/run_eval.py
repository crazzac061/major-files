import sys, glob, os, numpy as np, pandas as pd, torch
from ai.spec import OBS_DIM
from ai.envs.mock_backend import MockBackend
from ai.envs.slice_env import SliceEnv
from ai.rl.sac_ctde import SACCTDE
from ai.rl.normalizer import RunningNorm
from ai.rl.lstm_pred import TrafficPredictor
from ai.baselines import static_action, heuristic_action, random_action, _pack
from ai.evaluate import run_episode, sac_policy

torch.set_num_threads(1)
EVAL_SEEDS = range(9_100_000, 9_100_010)   # fresh seeds, never used for training or checkpoint choice
COLS = ["sla_rate", "ret", "c_urllc", "c_embb", "c_mmtc", "jain_inter", "mean_power"]

def priority_action(obs): return _pack(np.tile([0.5, 0.3, 0.2], (3, 1)))

def load_sac(d):
    ag = SACCTDE(); ag.load(f"{d}/best.pt")
    nm = RunningNorm(OBS_DIM); nm.load(f"{d}/norm.npz")
    pr = TrafficPredictor(); pr.net.load_state_dict(torch.load(f"{d}/lstm_best.pt")); pr.ready = True
    return ag, nm, pr

def main(run_dir):
    np.random.seed(0); rows = []
    env = SliceEnv(MockBackend())
    for name, fn in [("static", static_action), ("priority", priority_action),
                     ("heuristic", heuristic_action), ("random", random_action)]:
        for s in EVAL_SEEDS: rows.append({"ctrl": name, "model": -1, "seed": s, **run_episode(env, fn, s)})
    for d in sorted(glob.glob(f"{run_dir}/seed*")):
        if not os.path.exists(f"{d}/summary.json"): continue
        ag, nm, pr = load_sac(d); tr = int(d.split("seed")[-1])
        env_s = SliceEnv(MockBackend(), predictor=pr)
        for s in EVAL_SEEDS: rows.append({"ctrl": "sac", "model": tr, "seed": s, **run_episode(env_s, sac_policy(ag, nm), s)})
    df = pd.DataFrame(rows); df.to_csv("runs/eval_all.csv", index=False)
    pm = df.groupby(["ctrl", "model"])[COLS].mean().reset_index()
    print(pm.groupby("ctrl")[COLS].mean().round(3).to_string())
    sac = pm[pm.ctrl == "sac"]
    if len(sac): print("SAC models:", len(sac), " return min/max:", round(sac.ret.min(), 1), round(sac.ret.max(), 1))

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "runs/main")
