import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import sin, cos, atan2, sqrt, fabs, pi

def load_plans(file_path):
    plans = []
    i = -1
    path_x, path_y = [], []
    with open(file_path) as f:
      for line in f:
        print(line)
        
        if "Candidate" in line:
          i += 1
          plans.append([])
          continue

        data = line.split(',')
        if len(data) > 1:
          print(data)
          print(data[0], data[1])

          plans[i].append( [ float(data[0]), float(data[1]) ] )
    return plans

def print_plans(plans):
  for plan in plans:
    print("Plan: ")
    for cell in plan:
      print(cell[0], cell[1])

def get_timeline(t_range, unit_time):
  timeline = []
  curr_time = 0.0
  while curr_time < t_range:
    timeline.append(curr_time)
    curr_time += unit_time
  timeline.append(t_range)
  print(timeline)
  return timeline

def plot_plans_by_area(plans, s_range, t_range, unit_time):
  timeline = get_timeline(t_range, unit_time)

  ax = [None] * len(plans)

  fig = plt.figure(figsize=(5*len(plans),5))
  for i in range(len(plans)):
    ax[i] = fig.add_subplot(('1' + str(len(plans)) + str(i+1)))

  for i in range(len(plans)):
      ax[i].set_xlim(0, t_range)
      ax[i].set_ylim(0, s_range)

      for e in timeline:
          ax[i].axvline(e, ymax=1, color='k', linewidth=1) 

  colors = ['r', 'g', 'b', 'y']
  i = 0
  for plan in plans:
      for t, cell in enumerate(plan):
          if t + 1 < len(timeline):
            # Parallelogram
            x = [timeline[t], timeline[t], timeline[t+1], timeline[t+1]]
            y = [cell[0], cell[1], cell[1], cell[0]]
            ax[i].add_patch(patches.Polygon(xy=list(zip(x,y)), fill=True, color=colors[i]))
      i += 1
      
  plt.show()

#######################

plans = load_plans("../unitTesting/st_cell_plans.csv")

print_plans(plans)

plot_plans_by_area(plans, 100, 20, 1)
