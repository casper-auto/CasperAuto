import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import sin, cos, atan2, sqrt, fabs, pi

fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(111)

def load_st_areas(file_path):
  areas = []
  with open(file_path) as f:
    for line in f:
      print(line)
      line = line[:-1]
      data = line.split(',')
      print(data)
      s_in = float(data[0])
      t_in = float(data[1])
      s_out = float(data[2])
      t_out = float(data[3])
      length = float(data[4])
      p1 = [t_in, s_in - length / 2]
      p2 = [t_out, s_out - length / 2]
      p3 = [t_out, s_out + length / 2]
      p4 = [t_in, s_in + length / 2]
      area = [p1, p2, p3, p4, p1]
      areas.append(area)
  return areas

def plot_st_areas(st_areas, s_range, t_range, unit_time):
  ax.set_xlim(0, t_range)
  ax.set_ylim(0, s_range)

  for i, area in enumerate(st_areas):
    # Parallelogram
    x = [area[0][0], area[1][0], area[2][0], area[3][0]]
    y = [area[0][1], area[1][1], area[2][1], area[3][1]]
    ax.add_patch(patches.Polygon(xy=list(zip(x,y)), fill=True, color='r'))
      
  plt.show()

#######################

areas = load_st_areas("../unitTesting/st_graph_areas.csv")

plot_st_areas(areas, 100, 20, 1)
