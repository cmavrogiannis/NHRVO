#!/usr/bin/python3
import fileinput
import math
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PatchCollection

SCALE_FACTOR = 10

CAR_LENGTH, CAR_WIDTH = 0.3 * SCALE_FACTOR, 0.1 * SCALE_FACTOR

fig, ax = plt.subplots(1, 1, figsize=(7, 7))
ax.set_xlim(-15 * SCALE_FACTOR, 15 * SCALE_FACTOR)
ax.set_ylim(-15 * SCALE_FACTOR, 15 * SCALE_FACTOR)

frames = []
for line in fileinput.input():
    positions = line.split()
    frame = []
    for pos in positions[1:]:
        coords = pos.split(',')
        x = float(coords[0][1:]) * SCALE_FACTOR
        y = float(coords[1]) * SCALE_FACTOR
        th = math.degrees(float(coords[2][:-1]))
        frame.append((x, y, th))
    frames.append(frame)

num_cars = len(frames[0])
rects = []
for x, y, th in frames[0]:
    rects.append(plt.Rectangle((x, y), CAR_LENGTH, CAR_WIDTH, th))

collection = PatchCollection(rects, cmap=plt.cm.jet, alpha=0.4)
ax.add_collection(collection)

def update(frame):
    patches = []
    for i in range(num_cars):
        (x, y, th) = frame[i]
        rect = plt.Rectangle((x, y), CAR_LENGTH, CAR_WIDTH, th)
        patches.append(rect)

    collection.set_paths(patches)
    print(frame)

ani = FuncAnimation(fig, update, frames=frames, blit=False, interval=16)
plt.show()

