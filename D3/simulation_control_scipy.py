import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import scipy.linalg

# --- 1. 物理パラメータの設定 ---　数値が調節可能
g = 9.81      # 重力加速度 (m/s^2)
M = 1.0       # 台車の質量 (kg)
m = 0.1       # 振子の質量 (kg)
L = 0.5       # 振子の半長 (m) -> 全長 1.0m に対応
dt = 0.02     # 時間ステップ長 (s)
time_steps = 400 

# --- 2. 初期状態 ---  数値が調節可能
x = 0.0       # 台車の初期位置 (m)  0
x_dot = 0.0   # 台車の初期速度 (m/s)  0
theta = np.radians(20.0) # 初期の振子偏角 20度 (ラジアンに変換)  20
theta_dot = 0.0 # 振子の初期角速度 (rad/s)  0
# theta_dot = np.radians(30.0)

# --- 2. 动态构建线性化矩阵 A 和 B ---
denom = 4.0 * M + m

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -(3.0 * m * g) / denom, 0],
    [0, 0, 0, 1],
    [0, 0, (3.0 * g * (M + m)) / (L * denom), 0],
])

B = np.array([[0], [4.0 / denom], [0], [-3.0 / (L * denom)]])

# --- 3. 设定 Q 和 R 扣分权重 ---
Q = np.diag([20.0, 0.0, 10.0, 0.0])  # 状态惩罚
R = np.array([[1.0]])  # 控制能量惩罚

# --- 4. 解代数黎卡提方程，动态计算 K 值 ---
# A^T * P + P * A - P * B * R^-1 * B^T * P + Q = 0
P = scipy.linalg.solve_continuous_are(A, B, Q, R)

# 求解反馈矩阵 K = R^-1 * B^T * P
K = np.linalg.inv(R) @ B.T @ P

K_vec = K[0]

# 履歴の記録用
history_x = []
history_theta = []

b_pole = 0.05  # 轴承阻尼

# --- 4. 物理シミュレーションループ ---
for step in range(time_steps):
    # ------------------ 【重要な修正】LQRの前にマイナス符号を追加 ------------------
    # 制御フィードバックの方向を現在の物理エンジンの座標系に強制的に合わせる！

    # 【新增功能】：每隔 2.5 秒 (125 步) 施加一次脉冲干扰（模拟踢了摆杆一下）
    t = step * dt
    if step > 0 and step % 125 == 0:
        theta_dot += np.radians(20.0)  # 瞬时角速度冲击
    u = -np.dot(K_vec, [x, x_dot, theta, theta_dot])    
    u = np.clip(u, -50.0, 50.0) # 制御力（推力）を -50N から 50N の範囲に制限

    Sx = np.sin(theta)
    Cx = np.cos(theta)
    
    # 標準的なCartPole（台車と振子）の運動方程式
    temp = (u + m * L * (theta_dot**2) * Sx) / (M + m)
    
    # 1. 角加速度の計算
    # theta_ddot = (g * Sx - Cx * temp) / (L * (4.0/3.0 - (m * Cx**2) / (M + m)))
    # 在 physics loop 循环内部修改角加速度计算：
    # b_pole = 0.1  # 摆杆轴承的阻尼系数 (摩擦力)

    # 在求 theta_ddot 时，扣除由角速度产生的摩擦阻尼力矩:
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

# --- 5. アニメーションの可視化 ---
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
    cx = history_x[i] # 現在のフレームにおける台車の位置
    cy = 0.0          # 台車のY軸位置（床面上に固定）
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