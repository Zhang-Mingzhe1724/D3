import numpy as np
import scipy.linalg

# --- 1. 物理パラメータの設定 ---
g = 9.81      # 重力加速度 (m/s^2)
M = 1.0       # 台車の質量 (kg)
m = 0.1       # 振子の質量 (kg)
L = 0.5       # 振子の半長 (m) (全長 1.0m に対応)

# --- 2. 物理方程式に基づく線形化行列 A, B の構築 ---
# 分母項: 4*M + m
denom = 4.0 * M + m

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -(3.0 * m * g) / denom, 0],
    [0, 0, 0, 1],
    [0, 0, (3.0 * g * (M + m)) / (L * denom), 0],
])

B = np.array([[0], [4.0 / denom], [0], [-3.0 / (L * denom)]])

# --- 3. 重み行列 Q および R の設定 ---
# Q: 状態に対するペナルティ行列 [位置 x, 速度 x_dot, 角度 theta, 角速度 theta_dot]
Q = np.diag([20.0, 0.0, 10.0, 0.0])
# R: 制御入力（エネルギー）に対するペナルティ
R = np.array([[1.0]])

# --- 4. 代数リカッチ方程式 (ARE) の解法と K 値の算出 ---
# A^T * P + P * A - P * B * R^-1 * B^T * P + Q = 0
P = scipy.linalg.solve_continuous_are(A, B, Q, R)

# フィードバックゲイン行列の計算 K = R^-1 * B^T * P
K = np.linalg.inv(R) @ B.T @ P

print("================ 計算結果 (フィードバックゲイン K) ================")
print(f"k_x        = {K[0, 0]:.4f}")
print(f"k_xdot     = {K[0, 1]:.4f}")
print(f"k_theta    = {K[0, 2]:.4f}")
print(f"k_thetadot = {K[0, 3]:.4f}")