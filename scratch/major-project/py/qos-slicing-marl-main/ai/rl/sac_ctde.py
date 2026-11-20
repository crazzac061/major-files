import copy, numpy as np, torch, torch.nn as nn
from ai.spec import N_GNB, OBS_DIM, ACT_DIM, GAMMA

def mlp(i, o, h=256, n=3):
    L, d = [], i
    for _ in range(n): L += [nn.Linear(d, h), nn.ReLU()]; d = h
    return nn.Sequential(*L, nn.Linear(d, o))

class Actor(nn.Module):
    def __init__(self): super().__init__(); self.net = mlp(OBS_DIM, 2 * ACT_DIM)
    def forward(self, o, deterministic=False):
        mu, ls = self.net(o).chunk(2, -1); ls = ls.clamp(-5, 2); std = ls.exp()
        u = mu if deterministic else mu + std * torch.randn_like(mu)
        a = torch.tanh(u)
        logp = (-0.5 * ((u - mu) / std) ** 2 - ls - 0.5 * np.log(2 * np.pi)).sum(-1)
        logp = logp - torch.log(1 - a.pow(2) + 1e-6).sum(-1)
        return a, logp

class Critic(nn.Module):
    def __init__(self):
        super().__init__(); d = N_GNB * (OBS_DIM + ACT_DIM)
        self.q1, self.q2 = mlp(d, 1), mlp(d, 1)
    def forward(self, o, a):
        x = torch.cat([o.flatten(1), a.flatten(1)], -1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)

class SACCTDE:
    def __init__(self, lr=3e-4, tau=0.005, gamma=GAMMA, device="cpu"):
        self.dev, self.tau, self.gamma = device, tau, gamma
        self.actors = nn.ModuleList([Actor() for _ in range(N_GNB)]).to(device)
        self.critic = Critic().to(device); self.critic_t = copy.deepcopy(self.critic)
        self.log_alpha = torch.zeros(N_GNB, requires_grad=True, device=device)
        self.target_ent = -float(ACT_DIM)
        self.opt_a = torch.optim.Adam(self.actors.parameters(), lr=lr)
        self.opt_c = torch.optim.Adam(self.critic.parameters(), lr=lr)
        self.opt_al = torch.optim.Adam([self.log_alpha], lr=lr)

    def _all(self, o, det=False):
        out = [self.actors[i](o[:, i], det) for i in range(N_GNB)]
        return torch.stack([x[0] for x in out], 1), torch.stack([x[1] for x in out], 1)

    @torch.no_grad()
    def act(self, obs_norm, det=False):
        o = torch.as_tensor(obs_norm, dtype=torch.float32, device=self.dev).unsqueeze(0)
        return self._all(o, det)[0][0].cpu().numpy()          # (3, ACT_DIM) in [-1,1]

    def update(self, batch):
        o, a, r, o2 = batch
        alpha = self.log_alpha.exp().detach()
        with torch.no_grad():
            a2, lp2 = self._all(o2)
            q_t = torch.min(*self.critic_t(o2, a2))
            y = r + self.gamma * (q_t - (alpha * lp2).sum(1))
        q1, q2 = self.critic(o, a)
        c_loss = ((q1 - y) ** 2).mean() + ((q2 - y) ** 2).mean()
        self.opt_c.zero_grad(); c_loss.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), 10); self.opt_c.step()

        a_new, lp = self._all(o)
        a_loss = 0.0
        for i in range(N_GNB):                       # agent i's action replaced, others from buffer
            aj = a.clone(); aj[:, i] = a_new[:, i]
            q = torch.min(*self.critic(o, aj))
            a_loss = a_loss + (alpha[i] * lp[:, i] - q).mean()
        self.opt_a.zero_grad(); a_loss.backward()
        nn.utils.clip_grad_norm_(self.actors.parameters(), 10); self.opt_a.step()

        al_loss = -(self.log_alpha * (lp.detach() + self.target_ent).mean(0)).sum()
        self.opt_al.zero_grad(); al_loss.backward(); self.opt_al.step()

        with torch.no_grad():
            for p, pt in zip(self.critic.parameters(), self.critic_t.parameters()):
                pt.mul_(1 - self.tau).add_(self.tau * p)
        return dict(critic=c_loss.item(), actor=a_loss.item(), alpha=alpha.mean().item(),
                    q=q1.mean().item(), entropy=-lp.mean().item())

    def save(self, p): torch.save({"actors": self.actors.state_dict(), "critic": self.critic.state_dict(),
                                   "critic_t": self.critic_t.state_dict(), "log_alpha": self.log_alpha.data}, p)
    def load(self, p):
        d = torch.load(p, map_location=self.dev)
        self.actors.load_state_dict(d["actors"]); self.critic.load_state_dict(d["critic"])
        self.critic_t.load_state_dict(d["critic_t"]); self.log_alpha.data = d["log_alpha"]