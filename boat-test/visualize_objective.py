import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

from controller.scipy_logging_controller import ScipyLoggingController


params = (174.30346377634683, 40.96843387500243, 45.64218849294368, 33.43573042084108, 218.70396989880902, 210.6705153396057, 30.970283478964255, -6.0602203884899914, -16.805711037437604)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_zlim3d([0.0, 1e6])

df = pd.read_csv("logs/log.csv")
curr_idx = 0

m = ScipyLoggingController(False)

multiplier = 100
num_samples = 10

x = np.arange(-multiplier * m.a_max, multiplier * m.a_max, 2*multiplier * m.a_max / num_samples)
y = np.arange(-multiplier * m.max_alpha_mag, multiplier * m.max_alpha_mag, 2*multiplier * m.max_alpha_mag / num_samples)

x, y = np.meshgrid(x, y)
z_arr = []

x_r = np.ravel(x)
y_r = np.ravel(y)
for i in range(len(x_r)):
    z_arr.append(m.compute_objective([x_r[i], y_r[i]], *params))
z = np.array(z_arr)
z = z.reshape(x.shape)

ax.set_xlabel('linear accel')
ax.set_ylabel('angular accel')
ax.set_zlabel(r'distance value')

surf = [ax.plot_surface(x, y, z)]
surf[0].remove()


def animate(fn, p):
    # print(type(p))
    # p[0].remove()

    ax.clear()

    ax.set_zlim3d([0.0, 2.5e5])
    ax.set_xlabel('linear accel')
    ax.set_ylabel('angular accel')
    ax.set_zlabel(r'distance value')

    # load params at curr_idx
    params_series = df.iloc[fn % len(df)]
    params = tuple([params_series[k] for k in "theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy".split(", ")])

    z_arr = []
    for i in range(len(x_r)):
        z_arr.append(m.compute_objective([x_r[i], y_r[i]], *params))
    z = np.array(z_arr)
    z = z.reshape(x.shape)

    # p[0].remove()
    p = ax.plot_surface(x, y, z)
    return p,


ani = animation.FuncAnimation(fig, animate, fargs=(surf), interval=1, blit=False)
plt.show()


# theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy
# params = (174.30346377634683, 40.96843387500243, 45.64218849294368, 33.43573042084108, 218.70396989880902, 210.6705153396057, 30.970283478964255, -6.0602203884899914, -16.805711037437604)
#
# m = MinimalController(False)
# print(m.compute_objective([0, 0], *params))
#
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# # make data
#
# multiplier = 100
# num_samples = 100
#
# x = np.arange(-multiplier * m.a_max, multiplier * m.a_max, 2*multiplier * m.a_max / num_samples)
# y = np.arange(-multiplier * m.max_alpha_mag, multiplier * m.max_alpha_mag, 2*multiplier * m.max_alpha_mag / num_samples)
#
# x, y = np.meshgrid(x, y)
# z_arr = []
#
# x_r = np.ravel(x)
# y_r = np.ravel(y)
# for i in range(len(x_r)):
#     z_arr.append(m.compute_objective([x_r[i], y_r[i]], *params))
# z = np.array(z_arr)
# z = z.reshape(x.shape)
#
# ax.set_xlabel('linear accel')
# ax.set_ylabel('angular accel')
# ax.set_zlabel(r'distance value')
#
# # initial guess
# init = [-2.0, 3.0]
#
# # solved
# solved = [ -0.0869378503396574, 0.055280672040291634]
#
# ax.scatter([init[0]], [init[1]], [m.compute_objective(init, *params)], c='orange', depthshade=False)
# ax.scatter([solved[0]], [solved[1]], [m.compute_objective(solved, *params)], c='green', depthshade=False)
#
#
# surf = ax.plot_surface(x, y, z)
# plt.show()
