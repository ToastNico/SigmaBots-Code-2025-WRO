#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

gMotor = Motor(Port.A)
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)
sMotor = Motor(Port.D)
gyroSensor = GyroSensor(Port.S1)
colorSensor = ColorSensor(Port.S2)

robot = DriveBase(leftMotor, rightMotor, wheel_diameter=80, axle_track=104)

# Write your program here.
ev3.speaker.beep()

# SET VALUES DU PIPI KAKA
OrangeBalls = colorSensor.rgb(60, 25, 5)

PurpleBalls = colorSensor.rgb(25, 10, 50)

# Gobbler Startposition merken
gobbler_home = sMotor.angle()
sMotor.hold()
gobbler_close = gobbler_home + 90


purpleDetected : bool = False
isFull : bool = False

def PowerGobbler():
    if isFull:
        return

    sMotor.run(700)

def gobblerOpen():
    sMotor.stop()
    sMotor.run_target(600, gobbler_home)
    sMotor.hold()

def gobblerClose():
    sMotor.stop()
    sMotor.run_target(600, gobbler_close)
    sMotor.hold()

def shoot():
    gMotor.run_target(1000,45,stop.hold)
    wait(500)
    gMotor.run_target(600,45,stop.hold)


def driveUntilBall():
    detected_color = colorSensor.color
    detected_rgb   = colorSensor.rgb
    DUB = True

    while DUB:  #DUB = DriveUntilBall
        
        if detected_rgb in [colorSensor.rgb(OrangeBalls),colorSensor.rgb(PurpleBall)]:  # TODO put in the colours for the sensor
        ev3.speaker.beep()
        DUB = False

        if detected_color == Color.BLUE:
            detected_color = Color.RED
            return


        robot.straight(1000) 
        wait(10)

    
    leftMotor.stop(Stop.BRAKE)
    rightMotor.stop(Stop.BRAKE)

def recordColor():
    ballOrange() : bool = False
    ballPurple() : bool = False

    if colorSensor.rgb() = OrangeBalls:
        ballOrange = True

    if colorSensor.rgb() = PurpleBalls:
        ballPurple = True


    
def ClearLine1():
    PowerGobbler()
    driveUntilBall()
    if ballPurple:
        purpleCollected = True
        robot.straight(150)
        gobblerClose()
        robot.straight(-500)
        robot.straight(230)
        robot.turn(-90)
        robot.straight(200)
        gobblerOpen()
        robot.straight(-200)
        robot.turn(90)
        driveUntilBall()
        robot.straight(300)
        gobblerOpen()
        shoot()
        robot.straight(-1000)





#    if ballOrange:
        
        

ClearLine1()
