#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Stop, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
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
colorSensor = ColorSensor(Port.S2)
robot = DriveBase(leftMotor, rightMotor, wheel_diameter=80, axle_track=104)

COLOR_FILE = "colors.json"
OrangeBalls = (60, 25, 5)
PurpleBalls = (25, 10, 50)

ballOrange = False
ballPurple = False
purpleDetected = False
isFull = False

gobbler_home = sMotor.angle()
gobbler_close = gobbler_home + 90
sMotor.hold()

# ------------------ Farben speichern/laden ------------------ #

def load_colors():
    global OrangeBalls, PurpleBalls
    if COLOR_FILE in os.listdir():
        with open(COLOR_FILE, "r") as f:
            data = json.load(f)
            OrangeBalls = tuple(data["orange"])
            PurpleBalls = tuple(data["purple"])
            ev3.screen.print("Farben geladen.")
    else:
        ev3.screen.print("Standardfarben verwendet.")

def save_colors(orange, purple):
    with open(COLOR_FILE, "w") as f:
        json.dump({"orange": orange, "purple": purple}, f)

def calibrate_colors():
    ev3.screen.clear()
    ev3.screen.print("Halte Orange.\nDrücke CENTER.")
    while Button.CENTER not in ev3.buttons.pressed():
        rgb = colorSensor.rgb()
        ev3.screen.clear()
        ev3.screen.print("Orange:\nR:{}\nG:{}\nB:{}".format(*rgb))
        wait(100)
    orange = colorSensor.rgb()
    ev3.speaker.beep()
    wait(1000)

    ev3.screen.clear()
    ev3.screen.print("Halte Lila.\nDrücke CENTER.")
    while Button.CENTER not in ev3.buttons.pressed():
        rgb = colorSensor.rgb()
        ev3.screen.clear()
        ev3.screen.print("Lila:\nR:{}\nG:{}\nB:{}".format(*rgb))
        wait(100)
    purple = colorSensor.rgb()
    ev3.speaker.beep()
    wait(1000)

    save_colors(orange, purple)
    ev3.screen.clear()
    ev3.screen.print("Farben gespeichert.")

# ------------------ Mechanik & Bewegung ------------------ #

def PowerGobbler():
    if isFull:
        return
    sMotor.run(-1000)

def gobblerOpen():
    sMotor.stop()
    sMotor.run_target(600, gobbler_home)
    sMotor.hold()

def gobblerClose():
    sMotor.stop()
    sMotor.run_target(600, gobbler_close)
    sMotor.hold()

def shoot():
    gobblerOpen()
    gMotor.run_target(1000, -45, Stop.HOLD)
    wait(500)
    gMotor.run_target(600, 45, Stop.HOLD)

def color_distance(c1, c2):
    return sum((a - b) ** 2 for a, b in zip(c1, c2)) ** 0.5

def drive_straight_gyro(targetAngle, driveSpeed, distance_cm):
    robot.reset()
    correctionSpeed = 750
    while abs(robot.distance()) < abs(distance_cm):
        gyroAngle = gyroSensor.angle()
        error = targetAngle - gyroAngle
        df = 1.5 * (error / 90) - 0.5 * ((error / 90) ** 3)
        leftSpeed = driveSpeed - correctionSpeed * df
        rightSpeed = driveSpeed + correctionSpeed * df
        leftMotor.run(leftSpeed)
        rightMotor.run(rightSpeed)
        wait(10)
    stop_motors()

def stop_motors():
    leftMotor.stop(Stop.BRAKE)
    rightMotor.stop(Stop.BRAKE)

# ------------------ Farberkennung ------------------ #

def checkColor(threshold=25):
    rgb = colorSensor.rgb()
    dist_orange = color_distance(rgb, OrangeBalls)
    dist_purple = color_distance(rgb, PurpleBalls)
    if dist_orange < threshold:
        return "orange"
    if dist_purple < threshold:
        return "purple" 
    return "none"

# ------------------ Hauptlogik ------------------ #

def SortStartBalls():
    global purpleDetected, ballPurple, ballOrange

    drive_straight_gyro(0, 460, 315)
    stop_motors()
    color_result = checkColor()
    wait(3000)

    if color_result == "orange":
        ev3.screen.print("Orange")
        ev3.speaker.beep()
        wait(100)
        ev3.speaker.beep()
        ballOrange = True
        PowerGobbler()
        rightMotor.run(600)
        wait(1500)
        rightMotor.stop()
        sMotor.stop()

        color_result = checkColor()

        if color_result == "orange":
            PowerGobbler()
            drive_straight_gyro(0, 600, 100)
            sMotor.stop()
            gobblerClose()
            drive_straight_gyro(-90, 200, 0)
            rightMotor.run(500)
            wait(2000)
            rightMotor.stop()
            drive_straight_gyro(-90, 200, 0)
            drive_straight_gyro(0, 800, 400)
            drive_straight_gyro(0, -800, 100)
            drive_straight_gyro(-90, 200, 0)
            drive_straight_gyro(0, 800, 500)  # blue zone

        elif color_result == "purple":
            ev3.screen.print("Lila")
            ballPurple = True
            purpleDetected = True
            gobblerClose()
            rightMotor.run(700)
            wait(1000)
            rightMotor.stop()
            drive_straight_gyro(0, 800, 40)  # drop purple
            drive_straight_gyro(0, -800, 100)
            drive_straight_gyro(-180, 200, 0)
            drive_straight_gyro(0, 800, 500)  # blue zone

    elif color_result == "purple":
        ev3.screen.print("Lila")
        ev3.speaker.beep()
        ballPurple = True
        purpleDetected = True
        gobblerClose()
        drive_straight_gyro(0, 600, 50)
        drive_straight_gyro(120, 200, 0)
        drive_straight_gyro(0, 800, 500)
        drive_straight_gyro(0, 600, 100)
        drive_straight_gyro(-120, 200, 0)
        PowerGobbler()
        drive_straight_gyro(0, 800, 300)
        rightMotor.run(600)
        wait(2500)
        rightMotor.stop()
        gobblerClose()
        drive_straight_gyro(90, 200, 0)
        drive_straight_gyro(0, 800, 25)

    else:
        ev3.screen.print("Keine Farbe")
        PowerGobbler()
        drive_straight_gyro(0, 800, 200)
        sMotor.stop()
        drive_straight_gyro(-120, 200, 0)
        drive_straight_gyro(0, 600, 250)

# ------------------ Start ------------------ #

ev3.screen.clear()
ev3.screen.print("Warte 1s auf Knopf...")
watch = StopWatch()
watch.reset()

while watch.time() < 1000:
    if Button.UP in ev3.buttons.pressed():
        calibrate_colors()
        break
    wait(10)

load_colors()
SortStartBalls()
