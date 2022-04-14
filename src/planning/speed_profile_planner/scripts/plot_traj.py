import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch
from matplotlib import transforms

centerxs = []
centerys = []

# Create new Figure and an Axes which fills it.
fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(111)

def load_path(file_path):
    path_x, path_y = [], []
    with open(file_path) as f:

      for line in f:

        data = line.split()

        path_x.append(float(data[0]))
        path_y.append(float(data[1]))
    return path_x, path_y

ego_path_x, ego_path_y = load_path("../unitTesting/target_path.csv")

def load_traj(file_path):
    traj_x, traj_y, traj_t = [], [], []
    with open(file_path) as f:

      for line in f:

        data = line.split(',')

        traj_x.append(float(data[0]))
        traj_y.append(float(data[1]))
        traj_t.append(float(data[2]))

    return traj_x, traj_y, traj_t

car_traj_x, car_traj_y, car_traj_t = load_traj("../unitTesting/predicted_traj.csv")

def traj_point(center_x, center_y):
  centerxs.append(center_x)
  centerys.append(center_y)
  scat = ax.scatter(centerxs, centerys, s=100, color='y')

def update(frame_number):
    # Get an index which we can use to re-spawn the oldest raindrop.
    index = frame_number % len(car_traj_t)

    if frame_number % len(car_traj_t) == 0:
      centerxs.clear()
      centerys.clear()

    ax.clear()

    ax.axis('equal')
    ax.set_xlim(-20, 120)
    ax.set_ylim(-40, 0)

    ax.plot(ego_path_x, ego_path_y, 'bo')

    traj_point(car_traj_x[index], car_traj_y[index])

    plt.text(-10, -30, s="time="+str(index))


# Construct the animation, using the update function as the animation director.
animation = FuncAnimation(fig, update, interval=200)
plt.show()
