import time
import math
import matplotlib.pyplot as plt
from networktables import NetworkTables

ARM1 = "/doubleArm/arm1"
ARM2 = "/doubleArm/arm2"

NetworkTables.initialize(server="localhost")

plt.ion()
fig, ax = plt.subplots(1, 1)

while True:
  angle1 = NetworkTables.getEntry(ARM1 + "/sim/angle").getDouble(0) / 180 * 3.14
  angle2 = NetworkTables.getEntry(ARM2 + "/sim/angle").getDouble(0) / 180 * 3.14
  
  ax.cla()
  ax.set_aspect(1)
  ax.set_xlim(-2, 2)
  ax.set_ylim(-2, 2)
  ax.plot([0, 0], [0, 1.5], 'ko-')
  ax.plot([0, 1 * math.cos(angle1)], [0, 1 * math.sin(angle1)], 'bo-')
  ax.plot([1 * math.cos(angle1), 1 * math.cos(angle1) + 1 * math.cos(angle2 - 3.14 + angle1)], [1 * math.sin(angle1), 1 * math.sin(angle1) + 1 * math.sin(angle2 - 3.14 + angle1)], 'bo-')
  # ax.axvline( (0, 0), (0, 1.5), color="black")

  fig.canvas.draw()
  fig.canvas.flush_events()
  time.sleep(0.02)