é#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from threading import Thread
import sys

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
capteur = UltrasonicSensor(Port.S2)
capteur_arret = UltrasonicSensor(Port.S4)

watch = StopWatch()
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

vitesse = 2000  # Vitesse en degrés par seconde
left_motor.set_run_settings(vitesse,1500)
right_motor.set_run_settings(vitesse,1500)

def rotation_droite():
    robot.drive_time(100, -55, 1050)

def rotation_gauche():
    robot.drive_time(100, 55, 1050)

def motor_running_cocci():
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.D)
    while True:
        left_motor.run(20000)
        right_motor.run(20000)
        #if capteur.distance() < 500: 
            #left_motor.stop()
            #right_motor.stop()
        robot.straight(400)    
        wait(10)  # Attendre 100 millisecondes avant de reprendre la boucle
        rotation_gauche()
        robot.straight(1100)            
        rotation_droite()
        robot.straight(200)
def main_thread_cocci():
    touch_sensor = TouchSensor(Port.S1)
    t = Thread(target=motor_running_cocci)
    t.start()
    while True:
        ev3.speaker.beep(1000, 10)
        if touch_sensor.pressed():
            break
        if watch.time() >= 100000:  # Arreter apres 100 secondes
            break
    sys.exit()

while True :
    if capteur_arret.distance() < 200 :
        ev3.speaker.beep(0, 1)
    else : 
        break

watch.reset()
watch.resume()
while True:
    if watch.time() >= 90000:  # Attendre 90 secondes avant de demarrer le moteur
      break

main_thread_cocci()