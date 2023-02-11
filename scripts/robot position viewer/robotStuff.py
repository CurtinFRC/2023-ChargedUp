import math
import time

# Cartisian Plane relative
FRONT_LEFT_DISPLACEMENT = (-50, 50)
FRONT_RIGHT_DISPLACEMENT = (50, 50)
BACK_LEFT_DISPLACEMENT = (-50, -50)
BACK_RIGHT_DISPLACEMENT = (50, -50)


class Pose2D():
    def __init__(self, x_metre=0, y_metre=0, angle_deg=0):
        self.x = x_metre
        self.y = y_metre
        self.angle = angle_deg
        self.mode = "FIELD_PLANE"
    def ConvertToCartisianPlane(self):
        self.mode = "CARTISIAN_PLANE"
    def ConvertToFieldPlane(self):
        self.mode = "FIELD_PLANE"

def AddCoord(coord1, coord2):   return (coord1[0] + coord2[0], coord1[1] + coord2[1])

def Rotate(pivotCoord, rotatingCoord, angle):
    return (
    math.cos(math.radians(angle)) * (pivotCoord[0]-rotatingCoord[0]) - math.sin(math.radians(angle)) * (pivotCoord[1]-rotatingCoord[1]) + rotatingCoord[0],
    math.sin(math.radians(angle)) * (pivotCoord[0]-rotatingCoord[0]) + math.cos(math.radians(angle)) * (pivotCoord[1]-rotatingCoord[1]) + rotatingCoord[1]
    )

class Robot():
    def __init__(self, colour):
        self.position = (50, 50)
        self.angle = 0
        self.SetDefaultWheels()
        self.colour = colour
        self.lastCall = time.time()

    def SetDefaultWheels(self):
        self.front_left = AddCoord(self.position, FRONT_LEFT_DISPLACEMENT)
        self.front_right = AddCoord(self.position, FRONT_RIGHT_DISPLACEMENT)
        self.back_left = AddCoord(self.position, BACK_LEFT_DISPLACEMENT)
        self.back_right = AddCoord(self.position, BACK_RIGHT_DISPLACEMENT)
    def CalculateWheelPositions(self):
        self.SetDefaultWheels()
        self.front_left = Rotate((self.position[0], self.position[1]), self.front_left, self.angle)
        self.front_right = Rotate((self.position[0], self.position[1]), self.front_right, self.angle)
        self.back_left = Rotate((self.position[0], self.position[1]), self.back_left, self.angle)
        self.back_right = Rotate((self.position[0], self.position[1]), self.back_right, self.angle)

        self.front_midpoint = ((self.front_left[0] + self.front_right[0])/2, (self.front_left[1] + self.front_right[1])/2)
    
    def DrawRobot(self, pygame, screen):
        self.CalculateWheelPositions()
        pygame.draw.line(screen, self.colour, self.front_left, self.front_right)
        pygame.draw.line(screen, self.colour, self.back_left, self.back_right)
        pygame.draw.line(screen, self.colour, self.front_right, self.back_right)
        pygame.draw.line(screen, self.colour, self.back_left, self.front_left)
        pygame.draw.line(screen, self.colour, (self.position[0], self.position[1]), self.front_midpoint)


    def DriveRobot(self, speed, movementArray): # movementArray = [forward/Back, left/Right] <- [1/-1, -1/1]
        deltaTime = time.time() - self.lastCall
        if movementArray[0] != 0:
            self.position = (
                self.position[0] + movementArray[0] * speed * math.cos(math.radians(self.angle) * deltaTime),
                self.position[1] + movementArray[0] * speed * math.sin(math.radians(self.angle) * deltaTime)
            )

        if movementArray[1] != 0:
            self.position = (
                self.position[0] + speed * math.cos(math.radians(self.angle - 90 * movementArray[1]) * deltaTime),
                self.position[1] + speed * math.sin(math.radians(self.angle - 90 * movementArray[1]) * deltaTime)
            )




robotPositions = {
    "ActualPosition": Pose2D(),
    "IThinkIAmHere": Pose2D(),
    "VisionSaysIAmHere": Pose2D()
}