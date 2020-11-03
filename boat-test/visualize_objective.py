import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

from controller.minimal_controller import MinimalController


# theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy
params = (12.054533695911827, 10.029652879971882, 22.770182135850952, 385.876586537087, 39.6174771718745, 284.6028508312085, 0, -13.636367011796228, -14.763753039480862)

m = MinimalController(False)
print(m.compute_objective([0, 0], *params))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# make data

multiplier = 100
num_samples = 100

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

surf = ax.plot_surface(x, y, z)
plt.show()
