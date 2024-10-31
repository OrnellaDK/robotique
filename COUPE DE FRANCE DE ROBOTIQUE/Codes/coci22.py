#!/usr/bin/env pybricks-micropython
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

def motor_running_cocci():
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.D)
    while True:
        left_motor.run(1000)
        right_motor.run(1000)
        if capteur.distance() < 200: 
            left_motor.stop()
            right_motor.stop()
            wait(100)  # Attendre 100 millisecondes avant de reprendre la boucle
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
    if watch.time() >= 93000:  # Attendre 90 secondes avant de demarrer le moteur
       break

main_thread_cocci()