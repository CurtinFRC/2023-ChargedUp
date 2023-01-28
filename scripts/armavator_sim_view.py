import time
import math
import matplotlib.pyplot as plt
from networktables import NetworkTables

ARM_TOPIC = "/armavator/arm/sim/angle"
ELEVATOR_TOPIC = "/armavator/elevator/sim/height"

NetworkTables.initialize(server="localhost")

plt.ion()
fig, ax = plt.subplots(1, 1)

while True:
  angle = NetworkTables.getEntry("/armavator/arm/sim/angle").getDouble(0)
  height = NetworkTables.getEntry("/armavator/elevator/sim/height").getDouble(0)
  length = NetworkTables.getEntry("/armavator/arm/config/armLength").getDouble(0)
  maxHeight = NetworkTables.getEntry("/armavator/elevator/config/maxHeight").getDouble(0)

  ax.cla()
  ax.set_xlim(-1, 2)
  ax.set_ylim(-1, 2)
  ax.axhline(0)
  ax.axhline(1.981)
  ax.plot([0, 0], [0, maxHeight], 'ko-')
  ax.plot([0, length * math.cos(angle / 180 * 3.14)], [height, height + length * math.sin(angle / 180 * 3.14)], 'bo-')
  # ax.axvline( (0, 0), (0, 1.5), color="black")

  fig.canvas.draw()
  fig.canvas.flush_events()
  time.sleep(0.02)