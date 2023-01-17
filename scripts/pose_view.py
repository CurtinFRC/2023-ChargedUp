import argparse
import numpy as np
import time
import math
import pygame
from networktables import NetworkTables

parser = argparse.ArgumentParser("Pose View", description="Display poses on a model of the FRC Field")
parser.add_argument("--ip", default="10.47.88.2", help="IP Address of the Robot. Default: 10.47.88.2")
parser.add_argument("paths", type=str, nargs='+', help="The NetworkTable paths for the poses")

args = parser.parse_args()

NetworkTables.initialize(server=args.ip)
field_img = pygame.image.load("field.png")

pygame.init()
pygame.display.set_caption("Pose View")
screen = pygame.display.set_mode(field_img.get_size())

COLOURS = [(255, 0, 0), (0, 0, 255), (255, 0, 255), (0, 0, 0)]

running = True

def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def to_wh(meterXY, min_m=(0, 0), max_m=(16.459, 8.2296), reference_px=field_img.get_size()):
  return np.array([int(remap(meterXY[0], min_m[0], max_m[0], 0, reference_px[0])), int(remap(meterXY[1], min_m[1], max_m[1], 0, reference_px[1]))])

def to_xy(meterXY, min_m=(0, 0), max_m=(16.459, 8.2296), reference_px=field_img.get_size()):
  return np.array([int(remap(meterXY[0], min_m[0], max_m[0], 0, reference_px[0])), reference_px[1] - int(remap(meterXY[1], min_m[1], max_m[1], 0, reference_px[1]))])

def draw_robot(colour):
  module_positions = [NetworkTables.getEntry("/drivetrain/modules/" + str(i) + "/config/position").getDoubleArray([0, 0]) for i in range(1, 5)]
  minxy = np.array([min([p[0] for p in module_positions]), min([p[1] for p in module_positions])])
  maxxy = np.array([max([p[0] for p in module_positions]), max([p[1] for p in module_positions])])

  robot_dim = maxxy - minxy
  surface_dim = to_wh(robot_dim)

  surface = pygame.Surface(surface_dim, pygame.SRCALPHA)
  # surface.fill(colour)
  pygame.draw.rect(surface, colour, pygame.Rect(0, 0, surface_dim[0], surface_dim[1]), 5)
  pygame.draw.line(surface, colour, surface_dim // 2, (surface_dim // 2) + (surface_dim[0] // 4, 0), 5)

  for (i, pos) in enumerate(module_positions):
    pygame.draw.circle(surface, colour, to_xy(pos, minxy, maxxy, surface_dim), 25, 5)
  # pygame.draw.line(surface, colour, ())
  return surface

while running:
  # x = NetworkTables.getEntry(args.paths[0]).getDouble(None)
  # y = NetworkTables.getEntry(args.paths[1]).getDouble(None)
  
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False

  screen.blit(field_img, field_img.get_rect())

  if NetworkTables.isConnected():
    i = 0
    for path in args.paths:
      x = NetworkTables.getEntry(path + "/x").getDouble(4)
      y = NetworkTables.getEntry(path + "/y").getDouble(4)
      angle = NetworkTables.getEntry(path + "/angle").getDouble(0)

      colour = COLOURS[i % len(COLOURS)]

      rot_robot = pygame.transform.rotate(draw_robot(colour), angle)
      rot_robot_rect = rot_robot.get_rect()
      center = to_xy((x, y))
      screen.blit(rot_robot, (rot_robot_rect.x + center[0] - rot_robot_rect.width // 2, rot_robot_rect.y + center[1] - rot_robot_rect.height // 2))
    
      # ax.arrow(x, y, 0.5 * math.cos(angle), 0.5 * math.sin(angle), width=0.05)
      i += 1

  pygame.display.update()

  # fig.canvas.draw()
  # fig.canvas.flush_events()
  time.sleep(0.02)
