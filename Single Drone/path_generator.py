import numpy as np
import matplotlib.pyplot as plt


def linear_path(length: float = 1.0, num_points: int = 100, direction: np.ndarray = np.array([1., 0., 0.]), start_point: np.ndarray = np.array([0., 0., 0.])):
    # Normalize the direction vector
    direction_normalized = direction / np.linalg.norm(direction)

    # Calculate the end point of the linear path
    end_point = start_point + length * direction_normalized

    # Create the curve segment from start to end point
    curve_segment = np.linspace(start_point, end_point, num_points)

    return curve_segment



def circular_path(radius: float = 1.0, num_points: int = 100, center: np.ndarray = np.array([0., 0., 0.]), normal: np.ndarray = np.array([0., 0., 1.])):
    # Normalize the normal vector
    normal_normalized = normal / np.linalg.norm(normal)

    # Generate an orthogonal vector to the normal
    u, _, _ = np.linalg.svd(normal_normalized.reshape((3, 1)))
    orthogonal_vector = u[:, -1]

    # Generate angles for the circular path
    angles = np.linspace(0, 2 * np.pi, num_points)

    # Calculate the points on the circle using the normal and orthogonal vectors
    circle_points = center + radius * (np.cos(angles)[:, np.newaxis] * orthogonal_vector[np.newaxis, :] + np.sin(angles)[:, np.newaxis] * np.cross(normal_normalized, orthogonal_vector)[np.newaxis, :])

    return circle_points


def wave_path(amplitude: float = 1.0, wavelength: float = 2*np.pi, num_points: int = 100, start_point: np.ndarray = np.array([0., 0., 0.]), direction: np.ndarray = np.array([1., 0., 0.]), up: np.ndarray = np.array([0., 0., 1.])):
    # Calculate the displacement along the direction vector
    displacement = wavelength / (num_points - 1)

    # Calculate the points on the sine wave
    t = np.linspace(0, wavelength, num_points)
    x = start_point[0] + t * direction[0]
    y = start_point[1] + amplitude * np.sin(t)
    z = start_point[2] + t * direction[2] * np.cos(t)

    # Create the curve segment from the sine wave points
    curve_segment = np.column_stack((x, y, z))

    return curve_segment

def rpy_to_direction(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
    # Create the rotation matrix
    rotation_matrix = np.array([
        [np.cos(yaw) * np.cos(pitch),
         np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll),
         np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
        [np.sin(yaw) * np.cos(pitch),
         np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll),
         np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
        [-np.sin(pitch),
         np.cos(pitch) * np.sin(roll),
         np.cos(pitch) * np.cos(roll)]
    ])

    # Direction vector is the third column of the rotation matrix
    direction = rotation_matrix[:, 2]

    return direction


def main():
    # Create a figure and 3D axes
    fig = plt.figure()
    fig.canvas.mpl_connect('close_event', quit)
    ax = fig.add_subplot(111, projection='3d')
    path = circular_path(10.0, normal=rpy_to_direction(pitch=np.radians(0.0)), center=np.array([0, 0, 10]))
    print(path)
    # path = circular_path(10.0, normal=[1.0, 0.0 , 0.0])

    ax.plot(path[:, 0], path[:, 1], path[:, 2], '*--k',)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.legend()
    plt.show()

if __name__ == '__main__':
    main()



