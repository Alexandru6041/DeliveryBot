#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

import struct

# Declare motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
claw_motor = Motor(Port.B)

getDistance = UltrasonicSensor(Port.S3)
Color = ColorSensor(Port.S4)
Pressed = TouchSensor(Port.S1)
# Initialize variables.
# Assuming sticks are in the middle when starting.
right_stick_x = 124
left_stick_y = 124
open_claw = False
NULL = 0
i = 1.0
last_distance = -1
# A helper function for converting stick values (0 - 255)
# to more usable numbers (-100 - 100)


def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
 
    val: float or int
    src: tuple
    dst: tuple
 
    example: print(scale(99, (0.0, 99.0), (-1.0, +1.0)))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


# Find the PS3 Gamepad:
# /dev/input/event3 is the usual file handler for the gamepad.
# look at contents of /proc/bus/input/devices if it doesn't work.
infile_path = "/dev/input/event4"

# open file in binary mode
in_file = open(infile_path, "rb")

# Read from the file
# long int, long int, unsigned short, unsigned short, unsigned int
FORMAT = 'llHHI'
EVENT_SIZE = struct.calcsize(FORMAT)
event = in_file.read(EVENT_SIZE)
print("Running...")
# speaker.set_speech_options('ro', 'm2')
# speaker.set_volume(100, 'Beep')
# speaker.set_volume(100, 'PCM')

# speaker.beep(100, 300)
# speaker.say('vorbesc limba romana')

while event:
    (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)
    if ev_type == 3 and code == 3:
        right_stick_x = value
    if ev_type == 3 and code == 1:
        left_stick_y = value
    if ev_type == 1:
        if code == 305:
            claw_motor.dc(100)
            
        elif code == 308:
            claw_motor.dc(-100)
            
        elif code == 316:
            in_file.close()
            
    # Scale stick positions to -100,100
    forward = scale(left_stick_y, (0, 255), (100, -100))
    left = scale(right_stick_x, (0, 255), (100, -100))
    
    if(getDistance.distance() >= 300):
        left_motor.dc(forward - left)
        right_motor.dc(forward + left)
    
    if(getDistance.distance() < 300 and forward < 0):#
        left_motor.dc(forward - left)
        right_motor.dc(forward + left)
        
    elif(getDistance.distance() < 300 and getDistance.distance() > 5):
        left_motor.dc(33 * (forward - left) / 100)
        right_motor.dc(33 * (forward + left) / 100)
        # brick.light.on(Color.YELLOW)
    
    elif(getDistance.distance() == 5):
        left_motor.brake()
        right_motor.brake()

    if(getDistance.distance() is not last_distance):
        print(getDistance.distance())
        print("\n")
        last_distance = getDistance.distance()

    
    # Finally, read another event
    event = in_file.read(EVENT_SIZE)


in_file.close()
