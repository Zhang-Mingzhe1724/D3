import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy.linalg

# --- 1. 物理パラメータの設定 --- (調整可能)
g = 9.81      # 重力加速度 (m/s^2)
M = 1.0       # 台車の質量 (kg)
m = 0.1       # 振子の質量 (kg)
L = 0.5       # 振子の半長 (m) -> 全長 1.0m に対応
dt = 0.02     # 時間ステップ長 (s)
time_steps = 400 

# --- 2. 初期状態 --- (調整可能)
x = 0.0       # 台車の初期位置 (m)
x_dot = 0.0   # 台車の初期速度 (m/s)
theta = np.radians(25.0) # 初期の振子偏角 20度 (ラジアンに変換)
theta_dot = 0.0 # 振子の初期角速度 (rad/s)

# --- 3. 線形化行列 A および B の動的構築 ---
denom = 4.0 * M + m

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -(3.0 * m * g) / denom, 0],
    [0, 0, 0, 1],
    [0, 0, (3.0 * g * (M + m)) / (L * denom), 0],
])

B = np.array([[0], [4.0 / denom], [0], [-3.0 / (L * denom)]])

# --- 4. 重み行列 Q と R の設定 ---
Q = np.diag([20.0, 3.0, 10.0, 2.0])  # 状態ペナルティ行列
R = np.array([[1.5]])             # 制御エネルギーペナルティ

# --- 5. 代数リカッチ方程式 (ARE) による K 値の動的計算 ---
# A^T * P + P * A - P * B * R^-1 * B^T * P + Q = 0
P = scipy.linalg.solve_continuous_are(A, B, Q, R)

# フィードバックゲイン行列の計算 K = R^-1 * B^T * P
K = np.linalg.inv(R) @ B.T @ P
K_vec = K[0]

# 履歴記録用リスト
history_x = []
history_theta = []

# 【物理特性】：振子軸受の粘性摩擦係数（空気抵抗や回転軸の摩擦力）
b_pole = 0.05  

# 【現実感の再現】：1.25秒ごとの外乱衝撃のステップ間隔を算出 (1.25s / 0.02s = 62.5 ≒ 62ステップ)
impulse_interval = int(1.25 / dt)  

# --- 6. 物理シミュレーションループ ---
for step in range(time_steps):
    t = step * dt

    # 【外乱追加】：1.25秒ごとに振子へ角速度のパルス衝撃（手で突くような外乱）を加える
    if step > 0 and step % impulse_interval == 0:
        theta_dot += np.radians(20.0)  # 瞬時の角速度インパクト (+20 deg/s)

    # LQR制御入力の計算 (制御軸の向きを合わせるためにマイナス符号を付与)
    u = -np.dot(K_vec, [x, x_dot, theta, theta_dot])    
    u = np.clip(u, -50.0, 50.0)  # 制御入力（出力推力）を -50N から 50N の範囲に制限

    Sx = np.sin(theta)
    Cx = np.cos(theta)
    
    # 標準的な CartPole（台車と振子）の非線形運動方程式
    temp = (u + m * L * (theta_dot**2) * Sx) / (M + m)
    
    # 1. 角加速度の計算（軸受の回転摩擦・減衰トルク -b_pole * theta_dot を差引）
    theta_ddot = (g * Sx - Cx * temp - b_pole * theta_dot) / (L * (4.0/3.0 - (m * Cx**2) / (M + m)))
    
    # 2. 台車の加速度の計算
    x_ddot = temp - (m * L * theta_ddot * Cx) / (M + m)
    
    # --- 状態の更新 (オイラー法) ---
    x_dot += x_ddot * dt
    x += x_dot * dt
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt
    
    history_x.append(x)
    history_theta.append(theta)

# --- 7. アニメーションの可視化 ---
fig, ax = plt.subplots(figsize=(10, 5))

ax.set_xlim(-3.0, 3.0)
ax.set_ylim(-2.0, 2.0)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.6) 

# 背景の床面、台車、振子のグラフィック要素を定義
ground, = ax.plot([-4, 4], [-0.05, -0.05], 'k-', lw=1.5) 
cart, = ax.plot([], [], 's-', color='#1f77b4', markersize=12, lw=0) 
pole, = ax.plot([], [], 'o-', color='#d62728', lw=2, markersize=4)  

def init():
    cart.set_data([], [])
    pole.set_data([], [])
    return cart, pole

def animate(i):
    cx = history_x[i]          # 現在のフレームにおける台車の位置
    cy = 0.0                   # 台車のY軸位置（床面上に固定）
    current_theta = history_theta[i]
    
    # 振子の先端座標を計算
    px = cx + (2 * L) * np.sin(current_theta)
    py = cy + (2 * L) * np.cos(current_theta)
    
    # 描画データの更新
    cart.set_data([cx], [cy])
    pole.set_data([cx, px], [cy, py])
    return cart, pole

# アニメーションオブジェクトの作成
ani = animation.FuncAnimation(fig, animate, init_func=init,
                              frames=time_steps, interval=dt*1000, blit=True)

plt.title("倒立振子シミュレーションアニメーション (LQR制御)")
plt.show()