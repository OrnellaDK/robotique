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

#Initialisation de variable
vitesse1 = -1000
temps1 = 3200
vitesse2 = -750
temps2 = 5300
temps3 = 2700
rotation1 = 95
rotation2 = 95

# Fonction pour monter la pince
def monter_pince():
    elevateur.straight(180)

# Fonction pour descendre la pince
def descendre_pince():
    elevateur.straight(-180)

# Fonction qui déplace le robot avec une vitesse et un temps prédéfini(e)
def deplacement2(vitesse, temps):
    robot.drive_time(vitesse, 0, temps)
    if watch.time() > 99999:  #Arrêt après 100 secondes
            sys.exit()

#Fonction déplacement du robot
def code_robot():
    fermer_pince()
    deplacement2(vitesse1, temps1)
    rotation_droite(rotation1)
    wait(1000)
    ouvrir_pince()
    wait(1000)
    deplacement2(vitesse2, temps2)
    wait(1000)
    fermer_pince()
    wait(1000)
    rotation_droite(rotation2)
    wait(1000)
    deplacement2(vitesse1, temps3)

#Rotation du robot de -90°
def rotation_droite(rotation):
    robot.drive_time(50, -rotation, 2050)

#Rotation du robot de 90°
def rotation_gauche(rotation):
    robot.drive_time(50, rotation, 2050)

# Fonction pour fermer la pince
def fermer_pince():
    moteur_pince.run(-100)

# Fonction pour ouvrir la pince
def ouvrir_pince():
    moteur_pince.run(100)

#Fonction qui attrape des fleurs
def attrape_fleurs():
    robot.stop()
    wait(700)
    ouvrir_pince()
    wait(800)
    robot.straight(-170)
    wait(1000)
    fermer_pince()
    wait(1000)

#Fonction qui permet l'exécution de plusieurs fonctions en parallèle
def main_thread():
    t = Thread(target=code_robot) #Fonction parallèle
    t.start() #Démarrage de la fonction

    #Bouton d'arrêt d'urgence
    while True:
        if button_stop.pressed():
            break

    sys.exit()


#Démarrage code du robot
while True:
    watch.reset() #Le temps est remis à 0

    while button_stop.pressed() == False: #Condition démarrage du robot
        watch.resume() #Lancement du chronomètre
        main_thread() #Exécution de plusieurs fonctions en parallèle

        if watch.time() > 99999:  
            sys.exit()