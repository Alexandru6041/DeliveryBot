#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import threading

import struct

ev3 = EV3Brick()

LeftMotor = Motor(Port.A)
RightMotor = Motor(Port.D)
ClawMotor = Motor(Port.B)

Reader = ColorSensor(Port.S1) #color sensor
Button = TouchSensor(Port.S2) #touch sensor
InfraRed = InfraredSensor(Port.S3) #infrared sensor

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter = 55.5, axle_track = 104)#MASOARA AMPATAMENTUL SI ROTILE(DURLY)


# Write your program here.
right_stick_x = 124
right_stick_y = 124
DRIVESPEED = 100
speed = 0
turn = 0
running = True

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
 
    val: float or int
    src: tuple
    dst: tuple
 
    example: print(scale(99, (0.0, 99.0), (-1.0, +1.0)))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def scale_stick(value):
    return scale(value, (0, 255), (-100, 100))

def clamp(value, floor=-100, ceil=100):
    """
    Clamp the value within the floor and ceiling values.
    """
    return max(min(value, ceil), floor)

# Find the PS3 Gamepad:
# /dev/input/event3 is the usual file handler for the gamepad.
# look at contents of /proc/bus/input/devices if it doesn't work.
infile_path = "/dev/input/event3" #SYSTEM PATH TO CONTROLLER(DURLY)

# open file in binary mode
in_file = open(infile_path, "rb")

# Read from the file
# long int, long int, unsigned short, unsigned short, unsigned int
FORMAT = 'llHHI'
EVENT_SIZE = struct.calcsize(FORMAT)
event = in_file.read(EVENT_SIZE)


class MotorThread(threading.Thread):
    def __init__(self):
        # Add more sensors and motors here if you need them
        self.left_motor = LeftMotor
        self.right_motor = RightMotor
        threading.Thread.__init__(self)

    def run(self):
        print("Engine running!")
        # Change this function to suit your robot.
        # The code below is for driving a simple tank.
        turns = 0;
        
        while running:
            
            right_dc = clamp(-speed-turn)
            left_dc = clamp(-speed+turn)
            if(InfraRed.distance < 500):
                right_dc = right_dc / 2
                left_dc = left_dc / 2
            
            if(Button.pressed == True):
                break;
            
            self.right_motor.run_direct(duty_cycle_sp=right_dc)
            self.left_motor.run_direct(duty_cycle_sp=left_dc)


        self.motor.stop()

motor_thread = MotorThread()
motor_thread.setDaemon(True)
motor_thread.start()

while event:
    (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)
    if event.type == 3:
        if event.code == 1:
            Leftstick_Yvalue = scale(event.value, (0, 255),  (100, -100))
            Leftstick_Xvalue = scale(event.value, (0, 255), (100, -100))
            speed = scale_stick(event.value)

        if event.code == 0:    
            turn = scale_stick(event.value)
            
            
    if event.type == 1:
        if event.code == 317:
            ClawMotor.run_direct(100);        
                
    # Finally, read another event
    event = in_file.read(EVENT_SIZE)

in_file.close()
#red 5
#blue 2
#green 3
#yellow 4
# 