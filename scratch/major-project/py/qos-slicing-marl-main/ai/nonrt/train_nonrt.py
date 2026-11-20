import argparse, os, json, time, numpy as np, torch
from ai.spec import *
from ai.envs.mock_backend import MockBackend
from ai.envs.slice_env import SliceEnv
from ai.rl.sac_ctde import SACCTDE
from ai.rl.normalizer import RunningNorm
from ai.rl.lstm_pred import TrafficPredictor
from ai.reward import sla_met
from ai.nonrt.transformer_ppo import NonRTPolicy, window_features, ppo_update, gae, FEAT, WIN

NDEC, SUB = 60, NONRT_EVERY          # 60 decisions x 100 near-RT steps = 600 s

def load_near(d, ck="best"):
    ag = SACCTDE(); ag.load(f"{d}/{ck}.pt")
    nm = RunningNorm(OBS_DIM); nm.load(f"{d}/norm.npz")
    pr = TrafficPredictor(); pr.net.load_state_dict(torch.load(f"{d}/lstm_{ck}.pt")); pr.ready = True
    return ag, nm, pr

def run_ep(env, ag, nm, seed, policy=None, fixed=None, det=False):
    obs, _ = env.reset(seed=seed)
    hist = [np.zeros(FEAT, np.float32)] * WIN
    X, Z, LP, V, R, sc, sf = [], [], [], [], [], [], []
    for n in range(NDEC):
        x = torch.tensor(np.array(hist[-WIN:]))
        if fixed is not None:
            w = np.asarray(fixed, np.float32)
        else:
            with torch.no_grad():
                if det:
                    mu, _, v = policy(x[None]); w = torch.softmax(mu, -1)[0].numpy()
                else:
                    wt, z, lp, v = policy.sample(x[None]); w = wt[0].numpy()
                    X.append(x); Z.append(z[0]); LP.append(lp[0]); V.append(float(v[0]))
        env.set_w_a1(w)
        kp, cs = [], []
        for _ in range(SUB):
            obs, _, _, _, info = env.step((ag.act(nm(obs), det=True) + 1) / 2)
            kp.append(info["kpi"]); cs.append(info["c"])
        cs = np.array(cs)                                    # (100, 3 gNB, 3 slice)
        R.append(float(cs.mean())); sc.append(cs.mean((0, 1)))
        sf.append(float(np.mean([sla_met(c) for c in cs])))
        hist.append(window_features(np.array(kp)))
    last_v = 0.0
    if policy is not None and fixed is None and not det:
        with torch.no_grad(): last_v = float(policy(torch.tensor(np.array(hist[-WIN:]))[None])[2][0])
    return dict(mean_c=float(np.mean(R)), sla=float(np.mean(sf)), slice_c=np.mean(sc, 0),
                X=X, Z=Z, LP=LP, V=V, R=R, last_v=last_v)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--near", default="runs/wa1/seed0"); p.add_argument("--ck", default="best")
    p.add_argument("--mode", choices=["sens", "train"], default="train")
    p.add_argument("--iters", type=int, default=30); p.add_argument("--eps", type=int, default=4)
    p.add_argument("--eval-every", type=int, default=5); p.add_argument("--eval-eps", type=int, default=3)
    p.add_argument("--lr", type=float, default=1e-4); p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default="runs/nonrt")
    a = p.parse_args()
    torch.set_num_threads(2); torch.manual_seed(a.seed); np.random.seed(a.seed)
    ag, nm, pr = load_near(a.near, a.ck)
    env = SliceEnv(MockBackend(cyclic=True), predictor=pr, steps=NDEC * SUB)
    ev_seeds = [9_200_000 + k for k in range(a.eval_eps)]

    if a.mode == "sens":
        cases = [("equal", [1/3, 1/3, 1/3]), ("urllc-heavy", [.6, .2, .2]),
                 ("embb-heavy", [.2, .6, .2]), ("mmtc-heavy", [.2, .2, .6])]
        for name, w in cases:
            t0 = time.time()
            r = [run_ep(env, ag, nm, s, fixed=w) for s in ev_seeds]
            s3 = np.mean([x["slice_c"] for x in r], 0)
            print(f"{name:12s} mean_c {np.mean([x['mean_c'] for x in r]):.3f}  sla {np.mean([x['sla'] for x in r]):.3f}"
                  f"  urllc {s3[0]:.3f}  embb {s3[1]:.3f}  mmtc {s3[2]:.3f}  ({time.time()-t0:.0f}s)", flush=True)
        return

    pol = NonRTPolicy(); opt = torch.optim.Adam(pol.parameters(), a.lr)
    os.makedirs(a.out, exist_ok=True); log, best = [], -1.0
    for it in range(1, a.iters + 1):
        t0 = time.time(); X, Z, LP, ADV, RET, tc, ts = [], [], [], [], [], [], []
        for i in range(a.eps):
            e = run_ep(env, ag, nm, 1_000_000 + it * 100 + i, policy=pol)
            adv, ret = gae(e["R"], e["V"], e["last_v"])
            X += e["X"]; Z += e["Z"]; LP += e["LP"]; ADV += list(adv); RET += list(ret)
            tc.append(e["mean_c"]); ts.append(e["sla"])
        ppo_update(pol, opt, (torch.stack(X), torch.stack(Z), torch.stack(LP),
                              torch.tensor(np.array(ADV), dtype=torch.float32),
                              torch.tensor(np.array(RET), dtype=torch.float32)))
        row = dict(it=it, train_mean_c=float(np.mean(tc)), train_sla=float(np.mean(ts)), sec=round(time.time() - t0))
        if it % a.eval_every == 0:
            P = [run_ep(env, ag, nm, s, policy=pol, det=True) for s in ev_seeds]
            F = [run_ep(env, ag, nm, s, fixed=[1/3, 1/3, 1/3]) for s in ev_seeds]
            row.update(eval_mean_c=float(np.mean([x["mean_c"] for x in P])), eval_sla=float(np.mean([x["sla"] for x in P])),
                       fixed_mean_c=float(np.mean([x["mean_c"] for x in F])), fixed_sla=float(np.mean([x["sla"] for x in F])))
            if row["eval_mean_c"] > best: best = row["eval_mean_c"]; torch.save(pol.state_dict(), f"{a.out}/policy.pt")
        log.append(row); print(row, flush=True)
        json.dump(log, open(f"{a.out}/log.json", "w"))

if __name__ == "__main__":
    main()
