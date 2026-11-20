# Interface Spec v1.0.0

Per-gNB KPIs are normalised: throughput/target, delay/budget,
PRB util in [0,1], drop in [0,1].

- Near-RT step: 100 ms, episode 50 s = 500 steps, Non-RT every 100 steps
- Observation per gNB: 25 floats = kpi(12) + khat(6) + w_A1(3) + g_RIS(R=2) + p_energy(1) + ho(1)
- Action per gNB: 6 floats in [0,1] = w_BWP(3) + p_tx(1) + b_RIS(2)
- Agents: 3 (gNB 0, 1, 2), fixed order