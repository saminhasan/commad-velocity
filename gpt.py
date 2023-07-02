import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
from threading import Thread
import random
import time
from scipy.spatial import distance

# plt.ion()
inc = 0.01
inc_base = inc
error_max = 0
lim = 0.0
run = True
iter = 0.0
error_co = 0

def generate_helix(drone_id: int, number_of_drones: int, *args, **kwargs) -> np.ndarray:
    height_min = kwargs.get("height_min", 3)
    height_max = kwargs.get("height_max", 30)
    radius_min = kwargs.get("radius_min", 6.0)
    radius_max = kwargs.get("radius_max", 30.0)
    angular_step = kwargs.get("angular_step", 15.0)
    pitch = kwargs.get("pitch", 30.0)

    height_max -= height_min
    radius_max -= radius_min

    delta_phase = 360.0 / number_of_drones
    n_rotation = height_max / pitch

    theta_max = 360.0 * n_rotation
    radius_ratio = radius_max / radius_min
    theta = np.radians(np.arange(0, theta_max + 1, angular_step))

    x = (radius_min + radius_min * (radius_ratio * theta / theta[-1])) * np.cos(theta + (np.radians(delta_phase) * drone_id))
    y = (radius_min + radius_min * (radius_ratio * theta / theta[-1])) * np.sin(theta + (np.radians(delta_phase) * drone_id))
    z = theta * (height_max / (theta[-1]))
    z += abs(np.amin(z)) + height_min

    path_local = np.vstack([x, y, z]).T

    return path_local



def densify_curve(curve_points, num_points):
    x = curve_points[:, 0]
    y = curve_points[:, 1]
    z = curve_points[:, 2]
    tck, u = splprep([x, y, z], s=0)
    u_new = np.linspace(0, 1, num_points)
    dense_x, dense_y, dense_z = splev(u_new, tck)
    dense_curve = np.column_stack((dense_x, dense_y, dense_z))
    return dense_curve


def quit(arg):
    run = False

def play(fig, slider, run):

    try:
        # inc = 0.1
        global inc, inc_base, lim, error_co
        while run:
            error_coefficient = np.exp(-abs(error_max - lim)) if error_max >=lim else 1 
            inc = inc_base * error_coefficient 

            # t = slider.val + 0
            t = slider.val + inc

            if  t > 1.0 or t < 0.0:
                inc *=-1
                inc_base *=-1
            t = np.clip(t, 0.0, 1.0)
            # print(error_max, lim, error_max >=lim, error_coefficient, inc, t)
            print(error_co, error_max, inc)
            slider.eventson = False
            slider.set_val(t)
            slider.eventson = True
            update_position(None)
            time.sleep(0.1)
    except Exception:
        pass
    return

num_drones = 2
# sparse_curve_points = generate_expanding_spiral(num_sparse_points, radius, height)
sparse_curves = [generate_helix(drone_id, num_drones, pitch=15, height_max = 50) for drone_id in range(num_drones)] 

# Densify the curve segment
num_dense_points = 1000
dense_curves = [densify_curve(sparse_curve, num_dense_points) for sparse_curve in sparse_curves] 

# Create a figure and 3D axes
fig = plt.figure()
fig.canvas.mpl_connect('close_event', quit)
ax = fig.add_subplot(111, projection='3d')
# Plot the dense curve and sparse points
dense_lines = [ax.plot(dense_curve[:, 0], dense_curve[:, 1], dense_curve[:, 2], 'r-',) for dense_curve in dense_curves]
sparse_points = [ax.scatter(sparse_curve[:, 0], sparse_curve[:, 1], sparse_curve[:, 2], c='b', s=8) for sparse_curve in sparse_curves]

drones  = [ax.plot([dense_curve[0][0]], [dense_curve[0][1]], [dense_curve[0][2]], marker="o", color='g',markersize=5, label=f'Drone_{drone_id}') for drone_id, dense_curve in enumerate(dense_curves)]
drones_projection  = [ax.plot([dense_curve[0][0]], [dense_curve[0][1]], [dense_curve[0][2]], marker="o", color='k', markersize=5, label=f'Drone_{drone_id}') for drone_id, dense_curve in enumerate(dense_curves)]

# Create the slider
ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03])
slider = Slider(ax_slider, 'Position', 0, 1, valinit=0, valstep=0.01)

# Function to update the position of the moving point on the curve
def update_position(val):
    global inc, error_max, iter, error_co
    # print('update_position')
    error_max  = 0 
    parameter = slider.val
    point_index = int(parameter * (num_dense_points - 1))
    for dense_curve, drone in zip(dense_curves, drones):
        error_co = np.sin(iter)
        noise = error_co * np.random.random(size=dense_curve[point_index].shape)
        error = distance.euclidean(noise, np.array([0. , 0., 0.]))
        if error_max < error:
            error_max = error
        point = dense_curve[point_index] + noise
        drone[0].set_xdata([point[0]])
        drone[0].set_ydata([point[1]])
        drone[0].set_3d_properties([point[2]])
    fig.canvas.draw_idle()
    iter += 0.1

# Connect the slider to the update function
slider.on_changed(update_position)

# Set labels and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

t = Thread(target=play, args=[fig, slider, run])
t.start()
while run:
    plt.pause(1e-6)

t.join()


