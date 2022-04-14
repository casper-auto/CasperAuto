import matplotlib
import matplotlib.pyplot as plt

def load_path(file_path):
    path_x, path_y = [], []
    with open(file_path) as f:

      for line in f:

        data = line.split()

        path_x.append(float(data[0]))
        path_y.append(float(data[1]))
    return path_x, path_y

ego_path_x, ego_path_y = load_path("../unitTesting/target_path.csv")
obstacle_path_x, obstacle_path_y = load_path("../unitTesting/obstacle_path.csv")

# Create new Figure and an Axes which fills it.
fig = plt.figure(figsize=(15,10))
ax = fig.add_subplot(111)
ax.set_xlim(-20, 120)
ax.set_ylim(-40, 0)
plt.title('Ego and Obstacle displacement')
plt.grid(True)

ax.plot(ego_path_x, ego_path_y, 'bo')
ax.plot(obstacle_path_x, obstacle_path_y, 'r^')

plt.show()
