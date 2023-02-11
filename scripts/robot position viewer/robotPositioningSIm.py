import pygame
import sys
import robotStuff
import time

field_img = pygame.image.load("scripts/field.png")
pygame.init()
screen = pygame.display.set_mode((1024, 700))

robot = robotStuff.Robot((255, 0, 0))
screen.blit(field_img, field_img.get_rect())

robotAngle = 0
robotMoveSpeed = 1

angleChangeRate = 0.01

moveRequest = [0, 0]

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                robotAngle -= angleChangeRate
            elif event.key == pygame.K_RIGHT:
                robotAngle += angleChangeRate

            if event.key == pygame.K_w:
                moveRequest[0] = 1
            if event.key == pygame.K_s:
                moveRequest[0] = -1
            if event.key == pygame.K_a:
                moveRequest[1] = -1
            if event.key == pygame.K_d:
                moveRequest[1] = 1
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_w:
                moveRequest[0] = 0
            if event.key == pygame.K_s:
                moveRequest[0] = 0
            if event.key == pygame.K_a:
                moveRequest[1] = 0
            if event.key == pygame.K_d:
                moveRequest[1] = 0

    robot.angle = robotAngle
    robot.DriveRobot(robotMoveSpeed, moveRequest)
    #screen.blit(field_img, field_img.get_rect())
    pygame.display.update()
    robot.DrawRobot(pygame, screen)
    print(robot.position, robot.angle)
    pygame.display.update()
    time.sleep(0.01)
    pygame.draw.rect(screen, (101, 101, 110), (50, 50, 20, 20))