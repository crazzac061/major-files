import math, numpy as np, torch, torch.nn as nn
from ai.spec import N_GNB
from ai.reward import slice_scores

FEAT, WIN = 27, 12          # 3 gNB x 3 slices x (tput variance, violation share, PRB trend)

def sinusoid(n, d):
    pe = torch.zeros(n, d); pos = torch.arange(n).unsqueeze(1)
    div = torch.exp(torch.arange(0, d, 2) * (-math.log(10000.0) / d))
    pe[:, 0::2], pe[:, 1::2] = torch.sin(pos * div), torch.cos(pos * div); return pe

class NonRTPolicy(nn.Module):
    def __init__(self, d=64, heads=4, ff=256, layers=2):
        super().__init__()
        self.inp = nn.Linear(FEAT, d); self.register_buffer("pos", sinusoid(WIN, d))
        enc = nn.TransformerEncoderLayer(d, heads, ff, dropout=0.0, batch_first=True)
        self.enc = nn.TransformerEncoder(enc, layers, enable_nested_tensor=False)
        self.mu, self.v = nn.Linear(d, 3), nn.Linear(d, 1)
        self.log_std = nn.Parameter(torch.full((3,), -0.5))
    def forward(self, x):                                    # x: (B, WIN, FEAT)
        h = self.enc(self.inp(x) + self.pos).mean(1)         # mean-pooled
        return self.mu(h), self.log_std.exp(), self.v(h).squeeze(-1)
    def sample(self, x):
        mu, std, v = self(x); z = mu + std * torch.randn_like(mu)
        logp = (-0.5 * ((z - mu) / std) ** 2 - std.log() - 0.9189).sum(-1)
        return torch.softmax(z, -1), z, logp, v              # w_A1 = softmax(z)

def window_features(kpi_window):
    """kpi_window: (100, 3, 12) Near-RT KPIs over one 10-s window -> (27,)"""
    n = len(kpi_window)
    k = kpi_window.reshape(n, N_GNB, 3, 4)
    c = slice_scores(kpi_window.reshape(n, N_GNB, 12))                          # (n, gNB, slice)
    tput_var = k[..., 0].var(0)                                                 # (3,3)
    viol = (c < 0.999).sum(0) / n                                               # (3,3)
    prb = k[..., 2][-20:].mean(0) - k[..., 2][:20].mean(0)                      # trend
    return np.concatenate([tput_var.ravel(), viol.ravel(), prb.ravel()]).astype(np.float32)

def ppo_update(pol, opt, rollout, epochs=4, mbs=4, clip=0.2, vcoef=0.5, ent=0.01):
    x, z, old_lp, adv, ret = rollout
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    n = len(x)
    for _ in range(epochs):
        for idx in torch.randperm(n).chunk(mbs):
            mu, std, v = pol(x[idx])
            lp = (-0.5 * ((z[idx] - mu) / std) ** 2 - std.log() - 0.9189).sum(-1)
            ratio = (lp - old_lp[idx]).exp()
            pg = -torch.min(ratio * adv[idx], ratio.clamp(1 - clip, 1 + clip) * adv[idx]).mean()
            loss = pg + vcoef * (v - ret[idx]).pow(2).mean() - ent * std.log().sum()
            opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(pol.parameters(), 1.0); opt.step()

def gae(rews, vals, last_v, gamma=0.95, lam=0.95):
    adv, g = np.zeros(len(rews), np.float32), 0.0
    for t in reversed(range(len(rews))):
        nv = last_v if t == len(rews) - 1 else vals[t + 1]
        d = rews[t] + gamma * nv - vals[t]; g = d + gamma * lam * g; adv[t] = g
    return adv, adv + np.array(vals, np.float32)
