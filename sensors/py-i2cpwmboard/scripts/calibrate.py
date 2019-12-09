# Simple calibration tool using  the I2CPWMCONTROLLER servo controller library.
# Author: Bradan Lane Studio
# License: Public Domain

from __future__ import division
from __future__ import print_function

import sys
import time

# Import the module.
import i2cpwmcontroller


# to enable debug output: use DEBUG to see all the available logging messages
import logging
logging.basicConfig(level=logging.INFO)
#logging.basicConfig(level=logging.DEBUG)


try:
    from msvcrt import getch
except ImportError:
    def getch():
        """
        Gets a single character from STDIO.
        """
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)



# Initialise the controller using the default address (0x40).
controller = i2cpwmcontroller.I2CPWMCONTROLLER()

# Alternatively specify a different address and/or bus:
#controller = i2cpwmcontroller.I2CPWMCONTROLLER(address=0x41, busnum=0)

controller.set_pwm_frequency(50)

print ("")
print ("PWM SERVO CALIBRATION TOOL")
print ("")
print ("specify the servo number using '1 2 3 4 5 6 7 8 9 0 q w e r t y' to represent 1 thru 16")
print ("use 'g' to start the servo and 'h' to stop the servo")
print ("the initial center value for a servo is 300")
print ("use 'a' 's' 'd' 'f' to change the curent PWM value -10 -1 +1 +10 respectively")
print ("use 'x' to remember the most left / fastest reverse value")
print ("use 'c' to remember the stop / center value")
print ("use 'v' to remember the most right / fastest forward value")
print ("use '.' (the period) to exit the calibration program")
print ("")

center = 300
left = 300
right = 300
current = 300
servo = 1
run = False

controller.stop_servos()

while True:
    print("servo %2d      min=%3d current=%3d max=%3d      center=%3d travel=%3d"  % (servo, left, current, right, center, right-left), end='\r')
    sys.stdout.flush()

    key = getch() # this also returns the key pressed, if you want to store it

    if (key == "."):
        break

    if ((key == "1") or
        (key == "2") or
        (key == "3") or
        (key == "4") or
        (key == "5") or
        (key == "6") or
        (key == "7") or
        (key == "8") or
        (key == "9")):
        servo = int(key)
    if (key == "0"):
        servo = 10
    if (key == "q"):
        servo = 11
    if (key == "w"):
        servo = 12
    if (key == "e"):
        servo = 13
    if (key == "r"):
        servo = 14
    if (key == "t"):
        servo = 15
    if (key == "y"):
        servo = 16
        
    if (key == "g"):
        run = True
    if (key == "h"):
        run = False

    if (key == "x"):
        left = current
    if (key == "c"):
        center = current
    if (key == "v"):
        right = current

    if (key == "a"):
        current = current - 10
    if (key == "s"):
        current = current - 1
    if (key == "d"):
        current = current + 1
    if (key == "f"):
        current = current + 10

    if (run):
        controller.servo_absolute(servo, current)
    else:
        controller.stop_servos()



controller.stop_servos()
print ("")
print ("")
