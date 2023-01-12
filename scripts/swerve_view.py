import argparse
import time
import math
import matplotlib.pyplot as plt
from networktables import NetworkTables

parser = argparse.ArgumentParser("Swerve View", description="Visualisation of the Swerve Drive")
parser.add_argument("--ip", default="10.47.88.2", help="IP Address of the Robot. Default: 10.47.88.2")
parser.add_argument("root_path", default="swerve", type=str, help="The NetworkTable path for the swerve drive")

args = parser.parse_args()

NetworkTables.initialize(server="localhost")

def rotate(p, angle):
  return (p[0] * math.cos(angle) - p[1] * math.sin(angle), p[0] * math.sin(angle) + p[1] * math.cos(angle))

def draw_line(ax, p1, p2, *args, **kwargs):
  ax.plot([ p1[0], p2[0] ], [ p1[1], p2[1] ], *args, **kwargs)

plt.ion()
fig, ax = plt.subplots(1, 1)

table = NetworkTables.getTable(args.root_path)

while True:
  angle = table.getEntry("sim/angle").getDouble(0.0) / 180 * 3.14 + 3.14 / 2

  robot_pos = [
    table.getEntry("sim/x").getDouble(0.0), table.getEntry("sim/y").getDouble(0.0)
  ]
  robot_vel = rotate([
    table.getEntry("sim/vx").getDouble(0.0), table.getEntry("sim/vy").getDouble(0.0)
  ], angle)

  def tform(p):
    return [p[0] + robot_pos[0], p[1] + robot_pos[1]]

  module_pos = [
    tform(rotate(table.getEntry("modules/" + str(i) + "/config/position").getDoubleArray([0.0, 0.0]), angle)) for i in range(1, 5)
  ]
  module_angle = [
    angle + table.getEntry("modules/" + str(i) + "/angle").getDouble(0.0) / 180 * 3.14 for i in range(1, 5)
  ]
  module_speed = [
    table.getEntry("modules/" + str(i) + "/speed").getDouble(0.0) for i in range(1, 5)
  ]

  ax.cla()
  ax.set_xlim(-3 + robot_pos[0], 3 + robot_pos[0])
  ax.set_ylim(-3 + robot_pos[1], 3 + robot_pos[1])
  ax.set_aspect(1.0)

  for i in range(0, 4):
    circle = plt.Circle(module_pos[i], 0.3, color=(0.5, 0.5, 0.5));
    ax.add_patch(circle)
    mag = module_speed[i] / 10
    ax.arrow(module_pos[i][0], module_pos[i][1], mag * math.cos(module_angle[i]), mag * math.sin(module_angle[i]), width=0.05)
  
  draw_line(ax, module_pos[0], module_pos[1], color=(0.5, 0.5, 0.5))
  draw_line(ax, module_pos[1], module_pos[3], color=(0.5, 0.5, 0.5))
  draw_line(ax, module_pos[3], module_pos[2], color=(0.5, 0.5, 0.5))
  draw_line(ax, module_pos[2], module_pos[0], color=(0.5, 0.5, 0.5))

  ax.arrow(robot_pos[0], robot_pos[1], robot_vel[0] / 5, robot_vel[1] / 5, width=0.1)

  fig.canvas.draw()
  fig.canvas.flush_events()
  time.sleep(0.02)