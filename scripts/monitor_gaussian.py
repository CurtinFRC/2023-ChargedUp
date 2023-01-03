"""
A script used to plot the histogram output of sensors and their associated std deviation and 
mean. This script is primarily used to measure sensor intrinsics for filtering and pose estimation.

Author: Jaci Brunning
"""

import argparse
import numpy as np
import time
import matplotlib.pyplot as plt
from networktables import NetworkTables

parser = argparse.ArgumentParser("Monitor Gaussian", description="Monitor sensors for their gaussian parameters. Used to determining the std deviation for filtering methods.")
parser.add_argument("--ip", default="10.47.88.2", help="IP Address of the Robot. Default: 10.47.88.2")
parser.add_argument("-n", type=int, default="100", help="Number of readings to evaluate. Default: 100")
parser.add_argument("paths", type=str, nargs='+', help="The NetworkTable paths for the sensors to monitor")

args = parser.parse_args()

NetworkTables.initialize(server=args.ip)

readings = [
  np.zeros(args.n) for path in args.paths
]

i = 0

plt.ion()
fig, axs = plt.subplots(len(args.paths), 1, squeeze=False)

while True:
  for (idx, path) in enumerate(args.paths):
    reading = NetworkTables.getEntry(path).getDouble(None)
    if reading != None:
      readings[idx][i] = reading

      axs[idx][0].cla()
      axs[idx][0].set_title("{} std={:.5f}, mean={:.5f}".format(path, np.std(readings[idx]), np.mean(readings[idx])))
      axs[idx][0].hist(readings[idx])

  fig.canvas.draw()
  fig.canvas.flush_events()

  i += 1
  i = i % args.n
  
  time.sleep(0.02)

