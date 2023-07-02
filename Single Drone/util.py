import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
from threading import Thread
import random
import time
from scipy.spatial import distance

def densify_curve(curve_points, num_points):
    x = curve_points[:, 0]
    y = curve_points[:, 1]
    z = curve_points[:, 2]
    tck, u = splprep([x, y, z], s=0)
    u_new = np.linspace(0, 1, num_points)
    dense_x, dense_y, dense_z = splev(u_new, tck)
    dense_curve = np.column_stack((dense_x, dense_y, dense_z))
    return dense_curve

## untested
def point_to_parameter(curve_segment, point):
    # Calculate distances between point P and each point on the curve segment
    distances = np.linalg.norm(curve_segment - point, axis=1)

    # Find the index of the closest point
    closest_index = np.argmin(distances)


    # Normalize the distance to obtain the parameter value between 0 and 1
    # parameter = distances[closest_index] / np.max(distances)
    parameter = closest_index / curve_segment.shape[0]
    return parameter, closest_index, min(distances)

def parameter_to_index(number, array):
    number = np.clip(number, 0, 1)
    index = int(number * (array.shape[0] - 1))
    return index