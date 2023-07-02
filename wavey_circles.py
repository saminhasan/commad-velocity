import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.interpolate import splrep, splev

length_path_index = 15

phase = 0.0

theta = np.radians(np.linspace(0, 360 , length_path_index, endpoint=False))
theta = theta.reshape(length_path_index, 1)
# print(np.degrees(theta))
r1, r2, r3 = 5, 14, 20
x1, y1 = r1 * np.cos(theta), r1 * np.sin(theta)

x2, y2 = r2 * np.cos(theta), r2 * np.sin(theta)

x3, y3 = r3 * np.cos(theta), r3 * np.sin(theta)
'''
plt.ion()

'''


# Create subplot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.35)

axphase = plt.axes([0.25, 0.15, 0.65, 0.03])

phase_slider = Slider(axphase, 'Phase', 0.0, 2 * np.pi, phase)


a, = ax.plot(x1, y1, '-r')
b, = ax.plot(x2, y2, '-g')
c, = ax.plot(x3, y3, '-b')

print(x1.shape, y1.shape)
tck = splrep(x1, y1, s=3)

# Create a denser array of x coordinates for evaluation
# dense_x = np.linspace(min(x1), max(x1), 100)

# Evaluate the spline to get the corresponding y coordinates
# dense_y = splev(dense_x, tck)

# print(r1 * np.cos(phase), r1 * np.sin(phase))
d,   = ax.plot(r1 * np.cos(phase), r1 * np.sin(phase), '.r')
# e = ax.scatter(dense_x, dense_y)



def update(val):
    # print(val)
    # print(phase_slider.val)
    phase = phase_slider.val
    d.set_data(r1 * np.cos(phase), r1 * np.sin(phase))
    ax.axis('equal')
    fig.canvas.draw_idle()


phase_slider.on_changed(update)


ax.axis('equal')
plt.show()