"""
A script used to plot two variables against each other. This is useful for visualising the configuration space
of mechanisms.

Author: Jaci Brunning
"""

import argparse
import numpy as np
import time
import matplotlib.pyplot as plt
from networktables import NetworkTables

parser = argparse.ArgumentParser("Trace2d", description="Trace two variables against each other, such as for visualising configuration spaces")
parser.add_argument("--ip", default="10.47.88.2", help="IP Address of the Robot. Default: 10.47.88.2")
parser.add_argument("paths", type=str, nargs='+', help="The NetworkTable paths for the sensors to monitor (must be length of two")

args = parser.parse_args()

if len(args.paths) != 2:
  print("Must specify two paths!")
  exit(1)

NetworkTables.initialize(server=args.ip)

readingsX = [ ]
readingsY = [ ]

plt.ion()
fig, ax = plt.subplots(1, 1)

while True:
  x = NetworkTables.getEntry(args.paths[0]).getDouble(None)
  y = NetworkTables.getEntry(args.paths[1]).getDouble(None)
  
  if x != None and y != None:
    readingsX.append(x)
    readingsY.append(y)

  ax.cla()
  ax.plot(readingsX, readingsY)
  ax.set_xlabel(args.paths[0])
  ax.set_ylabel(args.paths[1])

  fig.canvas.draw()
  fig.canvas.flush_events()
  
  time.sleep(0.02)

