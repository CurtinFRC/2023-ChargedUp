import argparse
import numpy as np
import time
import math
import matplotlib.pyplot as plt
import lib.field as field
from networktables import NetworkTables

parser = argparse.ArgumentParser("Pose View", description="Display poses on a model of the FRC Field")
parser.add_argument("--ip", default="10.47.88.2", help="IP Address of the Robot. Default: 10.47.88.2")
parser.add_argument("paths", type=str, nargs='+', help="The NetworkTable paths for the poses")

args = parser.parse_args()

NetworkTables.initialize(server=args.ip)

plt.ion()
fig, ax = plt.subplots(1, 1)

COLOURS = ['b', 'r', 'k']

while True:
  # x = NetworkTables.getEntry(args.paths[0]).getDouble(None)
  # y = NetworkTables.getEntry(args.paths[1]).getDouble(None)
  
  ax.cla()
  field.drawField(ax)

  i = 0
  for path in args.paths:
    x = NetworkTables.getEntry(path + "/x").getDouble(0)
    y = NetworkTables.getEntry(path + "/x").getDouble(0)
    angle = NetworkTables.getEntry(path + "/x").getDouble(0) / 180 * 3.14

    ax.plot([x], [y], COLOURS[i % len(COLOURS)] + 'o')
    ax.arrow(x, y, 0.5 * math.cos(angle), 0.5 * math.sin(angle), width=0.05)
    i += 1

  fig.canvas.draw()
  fig.canvas.flush_events()
  
  time.sleep(0.02)

