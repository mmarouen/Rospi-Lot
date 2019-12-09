# Copyright (c) 2017 Bradan Lane Studio
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import division
import logging
import time
import math


PCA9685_ADDRESS    = 0x40

# drive mode servo positions

POSITION_LEFTFRONT  = 1
POSITION_RIGHTFRONT = 2
POSITION_LEFTREAR   = 3
POSITION_RIGHTREAR  = 4


logger = logging.getLogger(__name__)


class SERVO(object):
    def __init__(self, index=None, center=None, travel=None, direction=None, position=None):
        self.index = 0
        self.center = 0
        self.travel = 100
        self.direction = 1
        self.position = 0
        return self._config(index, center, travel, direction, position)

    def _config(self, index, center, travel, direction, position):
        if (index < 0) or (index > 16):
            logger.error("Invalid servo number %d :: servo numbers must be between 1 and 16", index)
            return None
        if index is not None:
            self.index = index
        if center is not None:
            self.center = center
        if travel is not None:
            self.travel = travel
        if direction is not None:
            self.direction = direction
        if position is not None:
            self.position = position
        return


        
class SERVOS(object):
    _servos = None
    
    def __init__(self, count=16):
        self._servos = [None] * count

    def get_servo(self, index=0):
        if (index < 0) or (index > 16):
            logger.error("Invalid servo number %d :: servo numbers must be between 1 and 16", index)
            return None
        return self._servos[index-1]

    def config_servo(self, index=0, center=None, travel=None, direction=None, position=None):
        if (index < 0) or (index > 16):
            logger.error("Invalid servo number %d :: servo numbers must be between 1 and 16", index)
            return None
        if self._servos[index-1] is None:
            servo = SERVO(index, center, travel, direction, position)
            self._servos[index-1] = servo
        else:
            self._servos[index-1].setall(index, center, travel, direction, positon)
        return

    def get_servo_position(self, index=0):
        if (index < 0) or (index > 16):
            logger.error("Invalid servo number %d :: servo numbers must be between 1 and 16", index)
            return 0
        if self._servos[index-1] is not None:
            return self._servos[index-1].position
        return 0

    def scan_positions(self, positions):
        # verify there is at least one server assigned to each required posiiton for the drive mode
        pos = [None] * positions
        # loop thru all of the servos and note their position (if they have one)
        for servo in self._servos:
            if servo is not None:
                if (servo.position > 0) and (servo.position <= positions):
                    pos[servo.position-1] = 1
        # now loop thru all requried positions to verify we found at least one servo
        for i in range(0, positions):
            if pos[i] is None:
                logger.error("Drive mode position #%d missing - Selected mode requires %d position assignments", i+1, positions)
                return False
        return True

    def proportional2absolute(self, index=0, value=0.0):
        if ((index<1) or (index>16)):
            logger.error("Invalid servo number %d :: servo numbers must be between 1 and 16", index)
            return 0
        if ((value < -1.0001) or (value > 1.0001)):
            logger.error("Invalid proportional %f :: proportional values must be between -1.00 and 1.0", value)
            return 0
        servo = self._servos[index-1]
        if (servo.center < 0) or (servo.travel < 0):
            logger.error("Missing servo configuration for servo[%d]", index)
            return 0
        return int((servo.direction * ((float(servo.travel) / 2) * value)) + servo.center)




# myboard = I2CPWMController.I2CPWMCONTROLLER(address, busnum)
# the module will attempt to autodetect the busnum with provided with'None'
# the default address is PCA9685_ADDRESS = 0x40 and the default bus is autodetected

class I2CPWMCONTROLLER(object):
                                
    # --------------------------------
    # properties
    # --------------------------------

    servos = None
    board = None
    # --------------------------------
    # constructor
    # --------------------------------

    def __init__(self, address=PCA9685_ADDRESS, **kwargs):
        if self.board is None:
            import Adafruit_PCA9685
            self.board = Adafruit_PCA9685.PCA9685(address, **kwargs)
        self.servos = SERVOS(16)
        return

    # --------------------------------
    # the following are public methods
    # --------------------------------

    def set_pwm_frequency(self, freq):
        """ comment """
        if ((freq<12) or (freq>1024)):
            logger.error("Invalid PWM frequency %d :: PWM frequencies should be between 12 and 1024", freq)
            freq = 50
        self.board.set_pwm_freq (freq)
        return


    def config_servo(self, index=0, center=0, travel=100, direction=1, position=None):
        """ comment """
        if (index < 0) or (index > 16):
            logger.error("Invalid servo number %d :: servo numbers are  between 1 and 16", index)
            return
        self.servos.config_servo(index, center, travel, direction, position)
        return


    def stop_servos(self, index=None):
        """ comment """
        if index is not None:
            self.servo_absolute(index, 0)
        else:
            self.board.set_all_pwm( 0, 0)
        return


    def servo_absolute(self, index=0, value=0):
        """ comment """
        if ((index<1) or (index>16)):
            logger.error("Invalid servo number %d :: servo numbers must be between 1 and 16", index)
            return;
        if ((value < 0) or (value > 4096)):
            logger.error("Invalid PWM value %d :: PWM values must be between 0 and 4096", value)
            return

        logger.debug("servo[%d] absolute position = %d", index, value)
        self.board.set_pwm(index-1, 0, value)
        return


    def servo_proportional(self, index=0, value=0.0):
        """ comment """
        if ((index<1) or (index>16)):
            logger.error("Invalid servo number %d :: servo numbers must be between 1 and 16", index)
            return;
        if ((value < -1.0001) or (value > 1.0001)):
            logger.error("Invalid proportional %f :: proportional values must be between -1.00 and 1.0", value)
            return

        logger.debug("servo[%d] proportional position = %f", index, value)
        absvalue = self.servos.proportional2absolute(index, value)
        self.servo_absolute(index, absvalue)


    def servos_drive(self, mode=None, linearx=0.0, lineary=0.0, angularz=0.0):
        """ comment """
        dir_x = -1.0 if (linearx  < 0) else 1.0
        dir_y = -1.0 if (lineary  < 0) else 1.0
        dir_r = -1.0 if (angularz  < 0) else 1.0

        temp_x = abs(linearx)
        temp_y = abs(lineary)
        temp_r = abs(angularz) # radians

        delta = temp_r

        speed = [0.0] * 4

        if (mode == "ackerman"):
            # with ackerman drive, steering is handled by a separate servo
            # we drive assigned servos exclusively by the linear.x

            speed[0] = temp_x * dir_x
            # speed[0] = self._convert_mps_to_proportional(speed[0])
            # if the result is greater that 1.0, we need to scale back to ±1.0
            if (abs(speed[0]) > 1.0):
                speed[0] = 1.0 * dir_x
            logger.debug("ackerman drive mode speed=%6.4f", speed[0])


        elif (mode == "differential"):
            # with differential drive, steering is handled by the relative speed of left and right servos
            # we drive assigned servos by mixing linear.x and angular.z
            # we compute the delta for left and right components
            # we use the sign of the angular velocity to determine which is the faster / slower
                
            if (dir_r > 0): # turnning right
                speed[0] = (temp_x + delta) * dir_x
                speed[1] = (temp_x - delta) * dir_x
            else:           # turning left
                speed[0] = (temp_x - delta) * dir_x
                speed[1] = (temp_x + delta) * dir_x

            logger.debug("computed differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1])

            # if any of the results are greater that 1.0, we need to scale all the results down
            travel = max(abs(speed[0]), abs(speed[1]))
            ratio = travel / 1.0
            if (ratio > 1.0):
                speed[0] /= ratio
                speed[1] /= ratio

            logger.debug("final differential drive mode speed left=%6.4f right=%6.4f", speed[0], speed[1])


        elif (mode == "mecanum"):
            # with mecanum drive, steering is handled by the relative speed of left and right servos
            # with mecanum drive, lateral motion is handled by the rotation of front and rear servos
            # we drive assigned servos by mixing linear.x and angular.z  and linear.y

            if (dir_r > 0): # turnning right
                speed[0] = speed[2] = (temp_x + delta) * dir_x
                speed[1] = speed[3] = (temp_x - delta) * dir_x
            else:           # turning left
                speed[0] = speed[2] = (temp_x - delta) * dir_x
                speed[1] = speed[3] = (temp_x + delta) * dir_x

            speed[0] += temp_y * dir_y
            speed[3] += temp_y * dir_y
            speed[1] -= temp_y * dir_y
            speed[2] -= temp_y * dir_y

            logger.debug("computed mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3])

            travel = max(max(max(abs(speed[0]), abs(speed[1])), abs(speed[2])), abs(speed[3]))
            ratio = travel / 1.0
            if (ratio > 1.0):
                speed[0] /= ratio
                speed[1] /= ratio
                speed[2] /= ratio
                speed[3] /= ratio

            logger.debug("final mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f", speed[0], speed[1], speed[2], speed[3])

        else:
            logger.error("drive mode not set or not recognized")
            return

        # find all drive servos and set their new speed
        for i in range(0, 16):
            # if we had a switch statement, we would use 'fall thru' to allow all necessary servos to be controlled
            if (mode == "mecanum"):
                if self.servos.get_servo_position(i+1) == POSITION_RIGHTREAR:
                    self.servo_proportional (i+1, speed[3])
                if self.servos.get_servo_position(i+1) == POSITION_LEFTREAR:
                    self.servo_proportional (i+1, speed[2])

            if (mode == "mecanum") or (mode == "differential"):
                if self.servos.get_servo_position(i+1) == POSITION_RIGHTFRONT:
                    self.servo_proportional (i+1, speed[1])

            if self.servos.get_servo_position(i+1) == POSITION_LEFTFRONT:
                self.servo_proportional (i+1, speed[0])
        return
            
