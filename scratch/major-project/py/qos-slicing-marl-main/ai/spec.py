SPEC_VERSION = "1.0.0"

N_GNB = 3
R = 2                         # RIS panels (keep 2; g_RIS is zeros when RIS is disabled)

K_DIM, KHAT_DIM, WA1_DIM = 12, 6, 3
OBS_DIM = K_DIM + KHAT_DIM + WA1_DIM + R + 1 + 1     # 25 for R=2 (decision D2)
ACT_DIM = 3 + 1 + R                                  # w_BWP[3], p_tx, b_RIS[R]

STEP_MS = 100
EPISODE_STEPS = 500           # 50 s
NONRT_EVERY = 100             # 10 s
W_FLOOR = 0.05                # starvation guard
GAMMA = 0.99

# kpi layout per gNB: slice s -> [tput, delay, prb, drop] at index s*4 + m
# all values are NORMALISED: tput/target, delay/budget (1.0 = exactly on target)
TPUT_DELAY_IDX = [0, 1, 4, 5, 8, 9]   # inputs/outputs of the LSTM