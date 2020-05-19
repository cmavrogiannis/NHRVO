#!/usr/bin/python3
import fileinput
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
plot, = plt.plot([], [], 'ro', markersize=1)

frames = []
for line in fileinput.input():
    positions = line.split()
    frame = { 'x' : [], 'y': [] }
    for pos in positions[1:]:
        coords = pos.split(',')
        x = float(coords[0][1:])
        y = float(coords[1][:-1])
        frame['x'].append(x)
        frame['y'].append(y)
    frames.append(frame)

def init():
    ax.set_xlim(-250, 250)
    ax.set_ylim(-250, 250)
    return plot,

def update(frame):
    plot.set_data(frame['x'], frame['y'])
    return plot,

ani = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=16)
plt.show()
