#!/usr/bin/env pybricks-micropython

# DELIVERY BOT

# Librării importate
from pybricks import ev3brick as ev3

from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

import struct, random, time


# Motoare si Senzori

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
claw_motor = Motor(Port.B)

distance = UltrasonicSensor(Port.S3)
color = ColorSensor(Port.S4)
pressed = TouchSensor(Port.S1)

# Variabile Default

# pozitia joystickurilor
right_stick_x = 124
left_stick_y = 124

open_claw = False # este cleștele deschis? Nu.
searching = False # caută parola de culoare? Nu.

bypass = False # treci peste incetinirea de siguranță folosind L2

last_distance = -1 # ultima distanță văzută de senzorul ultrasonic este -1, adică nu a văzut nimic


# Functie care scalaeaza valorile de la controller la valori intre -100 si 100, pentru motoare

def scale(val, src, dst):
    """
    val: float sau int
    src: tuple
    dst: tuple
 
    exemplu: print(scale(99, (0.0, 99.0), (-1.0, +1.0)))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

# Citire Input Controller

infile_path = "/dev/input/event4"
in_file = open(infile_path, "rb")
FORMAT = 'llHHI'
EVENT_SIZE = struct.calcsize(FORMAT)
event = in_file.read(EVENT_SIZE)

# Setarea Parolii de Culoare

ID = random.randint(0, 3)
color_list = ["VERDE", "ALBASTRU", "GALBEN", "ROȘU"]
colors = { 0: Color.GREEN, 1: Color.BLUE, 2: Color.YELLOW, 3: Color.RED}
print("Comanda dumneavoatră este în tranzit și va fi deblocată de culoarea " + color_list[ID] + "\n")

# Prelucrarea Inputurilor

while event:
    (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)
    
    #Inainte & Inapoi
    if ev_type == 3 and code == 3:
        right_stick_x = value
    
    #Rotatie Stanga & Dreapta
    if ev_type == 3 and code == 1:
        left_stick_y = value
    
    if ev_type == 1:
        
        #Butonul Triunghi pentru deschiderea forțată
        if code == 307:
            claw_motor.dc(100)
        
        #Butonul Patrat pentru inchiderea forțată 
        elif code == 308:
            claw_motor.dc(-100)
        
        #Butonul de PS pentru oprirea robotului
        elif code == 316:
            try:     
                in_file.close()
                break
            except ValueError:
                print("Delivery Bot Stopped")
        
        #Butonul Cerc pentru cererea autentificarii 
        elif code == 305:
            if searching == False:
                print("Comanda dumneavoastră a fost livrată și se așteaptă culoarea parolă ...")
            searching = True
        
        #Butonul X pentru anularea cererii de autentificare
        elif code == 304:
            if searching == True: 
                print('Comanda dumneavoastră nu a fost deblocată și se va intoarce la sediu.')
            searching = False
        
        #Pedala L2 
        elif code == 312:
            bypass = value
    
    #           
    forward = scale(left_stick_y, (0, 255), (100, -100))
    left = scale(right_stick_x, (0, 255), (100, -100))
    
    #Setarea distantelor de siguranta direct proportionale cu distanta pana la un obstacol dat
    if(distance.distance() >= 200):
        left_motor.dc(forward - left)
        right_motor.dc(forward + left)
    
    if(distance.distance() < 200 and forward < 0):#
        left_motor.dc(forward - left)
        right_motor.dc(forward + left)
        
    elif(distance.distance() < 200 and distance.distance() > 10 and bypass == False):
        left_motor.dc(33 * (forward - left) / 100)
        right_motor.dc(33 * (forward + left) / 100)
    
    elif(distance.distance() <= 50 and bypass == False):
        left_motor.brake()
        right_motor.brake()

    # Verificare Parola De Culoare
    if(pressed.pressed() == True):
        if(searching == False):
            print("Butonul de deblocare a fost apăsat și se așteaptă culoarea parolă ...")
        searching = True

    if(searching == True):
        if(color.color() == None):
            placeholder = True
            
        if(color.color() == colors[int(ID)]):
            claw_motor.dc(100)
            time.sleep(1)
            claw_motor.dc(-100)
            print("Comanda dumneavoatră a fost deblocată și ridicată! Enjoy!")
            
            searching = False
            
    # Citire Input Controller
    event = in_file.read(EVENT_SIZE)

in_file.close()
