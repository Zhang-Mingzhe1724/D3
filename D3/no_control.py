import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.animation as animation


# # --- 1. 物理参数设置 ---
# g = 9.81   # 重力加速度 (m/s^2)
# L = 1.0    # 摆杆长度 (m)
# dt = 0.02  # 每一个时间步长 (s)
# time_steps = 200 # 模拟多少步

# # --- 2. 初始状态 ---
# # 状态向量: [角度 theta, 角速度 omega]
# # 假设初始在上方偏右 1 度 (转换为弧度), 速度为 0
# theta = np.radians(1.0)
# omega = 0.0

# # 记录历史数据用于画图
# history_theta = []

# # --- 3. 物理仿真循环（欧拉法求解微分方程） ---
# for _ in range(time_steps):
#     # 核心物理公式: 计算角加速度
#     # alpha = (g / L) * sin(theta)
#     alpha = (g / L) * np.sin(theta)
    
#     # 更新状态
#     omega += alpha * dt  # 速度 = 速度 + 加速度 * 时间
#     theta += omega * dt  # 角度 = 角度 + 速度 * 时间
    
#     history_theta.append(theta)

# # --- 4. 动画可视化 ---
# fig, ax = plt.subplots(figsize=(5, 5))
# ax.set_xlim(-1.5, 1.5)
# ax.set_ylim(-1.5, 1.5)
# ax.set_aspect('equal')
# ax.grid(True)

# # 创建画布上的线条和点
# line, = ax.plot([], [], 'o-', lw=3, color=True, color='red') 

# def init():
#     line.set_data([], [])
#     return line,

# def animate(i):
#     # 根据当前角度计算摆球的 X, Y 坐标
#     # 注意：因为倒立摆向上是 0 度，所以 X 用 sin，Y 用 cos
#     current_theta = history_theta[i]
#     x = L * np.sin(current_theta)
#     y = L * np.cos(current_theta)
    
#     # 绘制从原点 (0,0) 到 (x,y) 的木杆
#     line.set_data([0, x], [0, y])
#     return line,

# ani = animation.FuncAnimation(fig, animate, init_func=init,
#                               frames=time_steps, interval=dt*1000, blit=True)

# plt.title("Inverted Pendulum Simulation (No Control)")
# plt.show()

# --- 1. 物理参数设置 ---
g = 9.81   # 重力加速度 (m/s^2)
L = 1.0    # 摆杆长度 (m)
dt = 0.02  # 每一个时间步长 (s)
time_steps = 200 # 模拟多少步

# --- 2. 初始状态 ---
# 状态向量: [角度 theta, 角速度 omega]
# 假设初始在上方偏右 1 度 (转换为弧度), 速度为 0
theta = np.radians(1.0)
omega = 0.0

# 记录历史数据用于画图
history_theta = []

# --- 3. 物理仿真循环（欧拉法求解微分方程） ---
for _ in range(time_steps):
    # 核心物理公式: 计算角加速度
    alpha = (g / L) * np.sin(theta)
    
    # 更新状态
    omega += alpha * dt  # 速度 = 速度 + 加速度 * 时间
    theta += omega * dt  # 角度 = 角度 + 速度 * 时间
    
    history_theta.append(theta)

# --- 4. 动画可视化 ---
fig, ax = plt.subplots(figsize=(5, 5))
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')
ax.grid(True)

# 【已修复】这里只保留一个 color 参数
line, = ax.plot([], [], 'o-', lw=3, color='red', markersize=10) 

def init():
    line.set_data([], [])
    return line,

def animate(i):
    current_theta = history_theta[i]
    x = L * np.sin(current_theta)
    y = L * np.cos(current_theta)
    
    line.set_data([0, x], [0, y])
    return line,

ani = animation.FuncAnimation(fig, animate, init_func=init,
                              frames=time_steps, interval=dt*1000, blit=True)

plt.title("Inverted Pendulum Simulation (No Control)")
plt.show()