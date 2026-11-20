import numpy as np, torch, torch.nn as nn, torch.nn.functional as F

class _Net(nn.Module):
    def __init__(self): super().__init__(); self.lstm = nn.LSTM(6, 64, 2, batch_first=True); self.head = nn.Linear(64, 6)
    def forward(self, x): o, _ = self.lstm(x); return self.head(o[:, -1])

class TrafficPredictor:
    """Predicts (tput, delay) x 3 slices, `horizon` steps ahead, from the last 10 steps.
    Shared across gNBs (gNB axis folded into the batch). Output is a fixed input to SAC (no SAC gradient)."""
    def __init__(self, horizon=2):
        self.h, self.net = horizon, _Net()
        self.opt = torch.optim.Adam(self.net.parameters(), 1e-3)
        self.eps, self.ready = [], False

    def add_episode(self, seq):                  # seq: (T, 3, 6)
        self.eps.append(np.asarray(seq, np.float32)); self.eps = self.eps[-50:]

    def train(self, steps=100, B=64):
        if not self.eps: return None
        for _ in range(steps):
            xs, ys = [], []
            for _ in range(B):
                e = self.eps[np.random.randint(len(self.eps))]
                t = np.random.randint(9, len(e) - self.h)
                xs.append(e[t - 9:t + 1]); ys.append(e[t + self.h])
            x = torch.tensor(np.array(xs)).permute(0, 2, 1, 3).reshape(-1, 10, 6)
            y = torch.tensor(np.array(ys)).reshape(-1, 6)
            loss = F.mse_loss(self.net(x), y)
            self.opt.zero_grad(); loss.backward(); self.opt.step()
        self.ready = True; return loss.item()

    @torch.no_grad()
    def predict(self, hist):                     # hist (3, 10, 6)
        return self.net(torch.as_tensor(hist, dtype=torch.float32)).numpy()

    def naive_vs_lstm(self, seqs):               # the "done" criterion for A5
        mse_l = mse_n = 0.0; n = 0
        for e in seqs:
            for t in range(9, len(e) - self.h):
                p = self.predict(e[t - 9:t + 1].transpose(1, 0, 2))
                mse_l += ((p - e[t + self.h]) ** 2).mean(); mse_n += ((e[t] - e[t + self.h]) ** 2).mean(); n += 1
        return mse_l / n, mse_n / n              # LSTM must be lowerS