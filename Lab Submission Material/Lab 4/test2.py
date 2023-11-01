# import plotting / math related libraries
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib import animation

def update_vector(i, v):
    U = [i, 2 * i, i / 2, 3 * i]
    V = [i / 2, i, 2 * i, 1]
    v.set_UVC(U, V) 
    fig.canvas.draw_idle()
    plt.pause(0.5)

X = [0, 0, 0, 0]
Y = [0, 0, 0, 0]
U = [0, 0, 0, 0]
V = [0, 0, 0, 0]

# plot
plt.ion()
fig = plt.figure()

ax = fig.add_subplot(projection='rectilinear')

colors = ['r', 'b', 'g', 'y']
labels = ['y-axis', 'x-axis', 'Strategy Vectors']
vector_labels = ['Go to Goal', 'Avoid Obstacle', 'Follow Wall cw', 'Follow Wall ccw']
v = ax.quiver(X, Y, U, V, color=colors, angles='xy', scale_units='xy', scale=1, width=0.003, label=labels)
plt.quiverkey(v, .1, .9, .2, vector_labels[0], color='r', labelpos='E')
plt.quiverkey(v, .1, .83, .2, vector_labels[1], color='b', labelpos='E')
plt.quiverkey(v, .1, .76, .2, vector_labels[2], color='g', labelpos='E')
plt.quiverkey(v, .1, .69, .2, vector_labels[3], color='y', labelpos='E')

legend1 = ax.legend(labels,
                    loc="lower left", title="Legend Types")
# set plot parameters
ax.set(xlim=[-400, 400], ylim=[-400, 400], xlabel='S_x [m]', ylabel='S_y [m]')

ax.axhline(y=0, color='k')
ax.axvline(x=0, color='k')
ax.grid(True)

# plt.draw()

# update plot
i = 0

for i in range(0, 1000):
    update_vector(i, v)
    i = i + 1

# fig.tight_layout()
# plt.show()

