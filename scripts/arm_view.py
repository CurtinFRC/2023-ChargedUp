import time
import math
import matplotlib.pyplot as plt
from networktables import NetworkTables

ARM_TOPIC = "/arm"

NetworkTables.initialize(server="localhost")

plt.ion()
fig, ax = plt.subplots(1, 1)

while True:
  angle = NetworkTables.getEntry(ARM_TOPIC + "/angle").getDouble(0)
  length = NetworkTables.getEntry(ARM_TOPIC + "/config/armLength").getDouble(0)
  
  ax.cla()
  ax.set_xlim(-1, 2)
  ax.set_ylim(-1, 2)
  ax.plot([0, length * math.cos(angle / 180 * 3.14)], [0, length * math.sin(angle / 180 * 3.14)], 'bo-')
  # ax.axvline( (0, 0), (0, 1.5), color="black")

  fig.canvas.draw()
  fig.canvas.flush_events()
  time.sleep(0.02)