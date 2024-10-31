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

#Configuration des ports de la brick Lego EV3
ev3 = EV3Brick()

moteur_pince = Motor(Port.B)
left_motor = Motor(Port.C)
right_motor = Motor(Port.D)

left_sensor = UltrasonicSensor(Port.S3)
right_sensor = UltrasonicSensor(Port.S1)
capteur_pince = UltrasonicSensor(Port.S2)
button_stop = TouchSensor(Port.S4)

#Fonction chronomètre
watch = StopWatch()

#Fonction pilotage du robot
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

#Déplacement en ligne droite pas à pas
def deplacement():
    for i in range(0,35):
        robot.straight(-50) #Le robot avance de 5 cm
        if watch.time() > 99999:  #Arrêt après 100 secondes
            sys.exit()
        
        #Détection d'obstacles
        while (capteur_pince.distance() < 100) | ((left_sensor.distance() < 170) | (right_sensor.distance() < 170)):
            robot.stop()
            wait(100)

#Fonction déplacement du robot
def code_robot():
    deplacement()    

#Fonction qui permet l'exécution de plusieurs fonctions en parallèle
def main_thread():
    t = Thread(target=code_robot) #Fonction parallèle
    t.start() #Démarrage de la fonction

    #Bouton d'arrêt d'urgence
    while True:
        if button_stop.pressed():
            break

    sys.exit() #Arrêt du code


#Démarrage code du robot
while True:
    watch.reset() #Le temps est remis à 0
    
    while button_stop.pressed() == False: #Condition démarrage du robot
        
        watch.resume() #Lancement du chronomètre
        main_thread() #Execution de plusieurs fonctions en parallèle
        
        if watch.time() > 99999:  
            sys.exit() 