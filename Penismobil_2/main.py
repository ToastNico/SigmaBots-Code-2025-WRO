#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pixycamev3.pixy2 import Pixy2
import json
import os

# Setup
ev3 = EV3Brick()
gMotor = Motor(Port.A)
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)
sMotor = Motor(Port.D)
gyroSensor = GyroSensor(Port.S1)
gyroSensor.reset_angle(0)
pixy2 = Pixy2(port=2, i2c_address=0x54)
robot = DriveBase(leftMotor, rightMotor, wheel_diameter=80, axle_track=104)

# Signature definitions
ORANGE_SIG = 2
PURPLE_SIG = 1
BLUE_SIG = 3

ballOrange = False
ballPurple = False
purpleDetected = False
isFull = False

gobbler_home = sMotor.angle()
gobbler_close = gobbler_home - 100
sMotor.hold()

globalTime = StopWatch()
globalTime.reset()

matchEndTime = 105000  # 1 minute 45 seconds in milliseconds

# ------------------ Pixy2 Detection Functions ------------------ #

def detectOrangeBall():
    """Check if orange ball is detected"""
    nr_blocks, blocks = pixy2.get_blocks(ORANGE_SIG, 1)
    return nr_blocks > 0

def detectPurpleBall():
    """Check if purple ball is detected"""
    nr_blocks, blocks = pixy2.get_blocks(PURPLE_SIG, 1)
    return nr_blocks > 0

def detectAnyBall():
    """Check if any ball (orange or purple) is detected"""
    return detectOrangeBall() or detectPurpleBall()

def checkColor():
    """
    Check what color ball is detected using Pixy2
    Returns: "orange", "purple", or "none"
    """
    if detectOrangeBall():
        return "orange"
    elif detectPurpleBall():
        return "purple"
    else:
        return "none"

# ------------------ Mechanik & Bewegung ------------------ #

def PowerGobbler(forward : bool):
    """Control gobbler intake"""
    global isFull
    if isFull:
        return
    if forward:
        sMotor.run(1000)
    else:
        sMotor.run(-1000)

def gobblerOpen():
    """Open the gobbler"""
    sMotor.stop()
    sMotor.run_target(600, gobbler_home)
    sMotor.hold()

def gobblerClose():
    """Close the gobbler"""
    sMotor.stop()
    target = sMotor.angle()
    gobblerClose = target - 100
    sMotor.run_target(600, gobblerClose)
    sMotor.hold()

def shoot():
    """Shoot mechanism"""
    gobblerOpen()
    gMotor.run_target(1000, -45, Stop.HOLD)
    wait(500)
    gMotor.run_target(600, 45, Stop.HOLD)

def drive_until_blue(targetAngle=0, driveSpeed=300):
    """
    Drives straight (with gyro) until Pixy2 sees a blue object (signature 3).
    """
    correctionSpeed = 750
    robot.reset()

    while True:
        # Check for blue ball (signature 3)
        nr_blocks, _ = pixy2.get_blocks(BLUE_SIG, 1)
        if nr_blocks > 0:
            ev3.screen.print("Blue found!")
            break

        # Gyro correction
        gyroAngle = gyroSensor.angle()
        error = targetAngle - gyroAngle
        df = 1.5 * (error / 90) - 0.5 * ((error / 90) ** 3)

        leftSpeed = driveSpeed - correctionSpeed * df
        rightSpeed = driveSpeed + correctionSpeed * df
        leftMotor.run(leftSpeed)
        rightMotor.run(rightSpeed)
        wait(10)

    stop_motors()


def drive_straight_gyro(targetAngle, driveSpeed, distance_cm):
    """Drive straight using gyro correction"""
    robot.reset()
    correctionSpeed = 750
    
    # Handle negative distances (reverse)
    if distance_cm > 0:
        direction = 1
    else:
        direction = -1
    abs_distance = abs(distance_cm)
    
    while abs(robot.distance()) < abs_distance:
        gyroAngle = gyroSensor.angle()
        error = targetAngle - gyroAngle
        df = 1.5 * (error / 90) - 0.5 * ((error / 90) ** 3)
        
        # Apply direction for forward/reverse
        leftSpeed = (driveSpeed * direction) - (correctionSpeed * df)
        rightSpeed = (driveSpeed * direction) + (correctionSpeed * df)
        
        leftMotor.run(leftSpeed)
        rightMotor.run(rightSpeed)
        wait(10)
    stop_motors()

def turn_to_angle(target_angle, speed=200):
    """Turns robot to an absolute angle using the gyro sensor."""
    tolerance = 2  # degrees
    max_speed = abs(speed)
    min_speed = 40  # to avoid stalling

    while True:
        current_angle = gyroSensor.angle()
        error = target_angle - current_angle

        if abs(error) <= tolerance:
            break

        # Handle angle wrapping for optimal turn direction
        if error > 180:
            error = error - 360
        elif error < -180:
            error = error + 360

        # Simple proportional control
        turn_speed = int(3 * error)

        # Limit speed
        if turn_speed > max_speed:
            turn_speed = max_speed
        elif turn_speed < -max_speed:
            turn_speed = -max_speed

        # Ensure it's not too slow
        if 0 < abs(turn_speed) < min_speed:
            if turn_speed > 0:
                turn_speed = min_speed
            else:
                turn_speed = -min_speed

        leftMotor.run(-turn_speed)
        rightMotor.run(turn_speed)

        wait(10)

    stop_motors()

def stop_motors():
    """Stop all drive motors"""
    leftMotor.stop(Stop.BRAKE)
    rightMotor.stop(Stop.BRAKE)

# ------------------ Hauptlogik ------------------ #

def SortStartBalls():
    global purpleDetected, ballPurple, ballOrange

    drive_straight_gyro(0, 460, 330)
    stop_motors()
    color_result = checkColor()
    wait(3000)

    if color_result == "orange":
        ev3.screen.print("Orange")
        wait(1000)
        ev3.speaker.beep()
        wait(100)
        ev3.speaker.beep()
        ballOrange = True
        PowerGobbler(False)
        drive_straight_gyro(0,400,130)
        wait(1000)
        sMotor.stop()
        gobblerClose()
        drive_straight_gyro(0,-80,130)
        turn_to_angle(90)
        drive_straight_gyro(90,400,200)
        checkColor()
        wait(3000)
        color2_result = checkColor()


        if color2_result == "orange":
            ev3.screen.print("Orange2")
            PowerGobbler(False)
            drive_straight_gyro(90, 600, 400)
            sMotor.stop()
            gobblerClose()
            turn_to_angle(-90)
            rightMotor.run(500)
            wait(2000)
            rightMotor.stop()
            turn_to_angle(-90)
            drive_straight_gyro(120, 800, 400)
            drive_straight_gyro(120, -800, 100)
            turn_to_angle(-90)
            drive_until_blue()  # blue zone

        elif color2_result == "purple":
            ev3.screen.print("Lila2")
            ballPurple = True
            purpleDetected = True
            gobblerClose()
            drive_straight_gyro(90,200,50)
            leftMotor.run(-400)
            wait(1000)
            leftMotor.stop()
            drive_straight_gyro(180, 600, 900)  # drop purple
            drive_straight_gyro(180, -80, 100)
            turn_to_angle(-90)
            wait(1000)
            turn_to_angle(-90)
            PowerGobbler(False)
            drive_until_blue()  # blue zone

    elif color_result == "purple":
        ev3.screen.print("Lila")
        ev3.speaker.beep()
        ballPurple = True
        purpleDetected = True
        gobblerClose()
        turn_to_angle(120)
        drive_straight_gyro(120, 800, 500)
        drive_straight_gyro(120, -200, 100)
        turn_to_angle(-120)
        PowerGobbler(False)
        drive_straight_gyro(0, 800, 300)
        rightMotor.run(600)
        wait(2500)
        rightMotor.stop()
        gobblerClose()
        turn_to_angle(90)
        drive_until_blue()

    else:
        ev3.screen.print("Keine Farbe")
        PowerGobbler(False)
        drive_straight_gyro(0, 800, 200)
        sMotor.stop()
        turn_to_angle(120)
        drive_straight_gyro(120, 600, 250)

        checkColor()
        wait(1000)
        color3_result = checkColor()

        if color3_result == "orange":
            turn_to_angle(-10)
            PowerGobbler(False)
            drive_straight_gyro(-130,500,200)
            drive_straight_gyro(-130,-80,50)
            gobblerClose()
            turn_to_angle(-90)
            rightMotor.run(500)
            wait(5000)
            rightMotor.stop()
            drive_straight_gyro(0,800,350)
            



        elif color3_result == "purple":
            pass


def midGameLoop():
    seeBlue = False

    drive_straight_gyro(0, -100, 1000) #Zuruck zur wand
    #
    # Erzaets mit reset pos
    #

    drive_straight_gyro(0, 600, 50)
    turn_to_angle(90)
    drive_straight_gyro(90, 600, 50)
    turn_to_angle(0)

    gobblerClose()
    while seeBlue == False:
        if checkColor() == "blue":
            seeBlue = True
        drive_straight_gyro(0, 600, 100)
    
    PowerGobbler(False)

    drive_straight_gyro(0, -100, 1000)
    drive_straight_gyro(0, 600, 50)
    turn_to_angle(-90)
    drive_straight_gyro(-90, 600, 200)
    turn_to_angle(0)

    gobblerClose()
    while seeBlue == False:
        if checkColor() == "blue":
            seeBlue = True
        drive_straight_gyro(0, 600, 100)
    
    PowerGobbler(False)

    drive_straight_gyro(0, -100, 1000)
    drive_straight_gyro(0, -100, 100)
    turn_to_angle(90)
    drive_straight_gyro(90, 600, 50)
    turn_to_angle(0)
    drive_straight_gyro(0, -100, 100) #geh zu mitte dammit man die function recursibely callen kann

    #TODO: if mit richtigem erzaetzen

    if globalTime.time() < matchEndTime:  # 1 minute 50 seconds
        midGameLoop()
    else:
        endGame()

def endGame():
    gobblerClose()
    while seeBlue == False:
        if checkColor() == "blue":
            seeBlue = True
        drive_straight_gyro(0, 600, 100)
    
    gobblerOpen()
    shoot()
    drive_straight_gyro(0, -100, 1000)  # Zurück zur Wand

# ------------------ Start ------------------ #

ev3.screen.clear()
ev3.screen.print("Pixy2 bereit!")
ev3.screen.print("Starte Programm...")

# Turn on Pixy2 lamp for better detection
pixy2.set_lamp(1, 0)

try:
    SortStartBalls()
    ev3.screen.print("Programm beendet")
except Exception as e:
    print("Fehler:", str(e))
    ev3.speaker.beep()
    ev3.speaker.beep()
finally:
    # Turn off Pixy2 lamp when done
    pixy2.set_lamp(0, 0)