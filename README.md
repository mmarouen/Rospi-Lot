# Python I2CPWMCONTROLLER Package
Python code to use the I2C PWM servo controller with a Raspberry Pi.
The library provides an interface to PWM servos. _(see documentation below)_

## Installation
To install the library, run the following commands on a Raspberry Pi:

```
sudo apt-get install git build-essential python-dev python3-pip
cd ~
git clone https://gitlab.com/bradanlane/py-i2cpwmboard.git
cd py-i2cpwmboard/src
sudo python3 setup.py install
```

The package will run under python (2.7) and python3 (3.3 .. 3.5). The documentation assumes the use of python3

## LoCoRo Examples
The I2C PWM Controller is used by the LoCoRo project. To install the LoCoRo examples, run the following commands on a Raspberry Pi:

```
cd ~
git clone https://gitlab.com/bradanlane/py-locoro.git
cd py-locoro
```
See `https://gitlab.com/bradanlane/py-locoro` and `http://stemroller.com` for more information on the LoCoRo project.


## API Programming Documentation

#### Import Package
Before using the I2C PWM controller package, you must import it into your python program.

```
import i2cpwmcontroller
```

#### Create the controller object
The Adafruit PCA9685 boards have a default I2C address of 0x40. Multiple boards may be used together. 
Each board connected to the Raspberry Pi must have its own address.
The board address may be set on each board using the A0 .. A5 solder tabs.

The Raspberry Pi 3 used `i2c-1`. This is referred to as 'bus number 1'.
The library will attempt to auto-detect the bus number.
For _Pi-like_ boards, the bus number may be different.

From a command prompt, test the I2C device on bus 1 using the command `i2cdetect -y 1`. If an error is generated, try using the command `i2cdetect -y 0`

The `i2cdetect` command will display a table of detected device addresses. The default address for the I2C PWM board is 0x40.(The address 0x70 is the Raspberry Pi, itself.)

From the API, the error `IOError: [Errno 2] No such file or directory: '/dev/i2c-?'` indicates the board is on a different bus number than the one being auto-detected or explicitly assigned.
If the auto-detect code does not correctly identify the I2C bus,
then it is necessary to manually specify the bus number when initializing the library.

```
# creates a controller with default settings
controller = i2cpwmcontroller.I2CPWMCONTROLLER()

# creates a controller for a board with a specific address
# and explicitly specify the bus number 
controller = i2cpwmcontroller.I2CPWMCONTROLLER(address=0x41, busnum=0)
```

#### Set the PWM Frequency
The standard PWM frequency for analog servos is 50Hz and digital servos use either 125Hz or 250Hz.

```
controller.set_pwm_frequency(50)
```

#### Configure Servos
PWM servos have a center and min/max travel range.

Small variances in the manufacturing of servos means they may not all be exactly the same.
To account for the differences in servos, the API supports a configuration for each servo.

Servos need to be configured before using `servo_drive()` or `servo_proportional()`.
Servos do not need to be configured before using `servo_absolute()`.

Servo configuration parameters consists of the __servo number__ (1 .. 16), the __center value__, the __travel range__, a __direction__ setting (helpful to reverse the direction of a servo), and the servo __position__ within the drive system (use 0 for servos not used by the system).

The applicable servos are assigned positions as follows:
  
    position | ackerman | differential | mecanum
    --------|----------|--------------|--------
    position 1 corresponds to | drive | left | left-front
    position 2 corresponds to | | right | right-front
    position 3 corresponds to | | | left-rear
    position 4 corresponds to | | | right-rear


_There is a configuration program included in the `examples` folder to help determine configuration settings for each servo.
Use the program to determine the PWM value for the center/zero position and the min/max which determines the travel range of the servo._


```
# example configure two servos for differential drive
# since the servos are mounted opposite each other, we set the direction of one to be negative
controller.config_servo(1, 339, 100, -1, 1)
controller.config_servo(2, 338, 100,  1, 2)

# example configuration for a standard servo which travels ±90 degrees
controller.config_servo(3, 334, 108,  1, 0)
```

#### Set Servo Absolute Pulse
Setting  a standard servo PWM value will set the position of the servo arm within it's full range of travel of  ±90 degrees.
Setting  a continuous rotation servo PWM value will set the speed of the servo within it's full 
range of maximum forward and reverse.

Servo absolute parameters consists of the __servo number__ (1 .. 16) and a __PWM value__ (0 .. 4096).


_The calibration program in the `examples` folder uses absolute movement._

```
# set servo #1 pulse value to 350
controller.servo_absolute(3, 350)
```

#### Move Servo
Moving a standard servo will set the position of the servo arm within it's full range of travel of  ±90 degrees.
Moving a continuous rotation servo sets the speed of the servo within it's full range of maximum forward and reverse. Proportional movement uses an input value of ±1.0.

Servo proportional parameters consists of the __servo number__ (1 .. 16) and a __proportional value__ (-1.0 .. 1.0).


```
# move servo #3 45 degrees left (assumes servo #3 is a standard servo)
# note: left is negative and right is positive; 45 degrees is 50% of 90 degrees
controller.servo_proportional(3, -0.5)

# move servo #1 at full speed forward (assumes servo #1 is a continuous rotation servo)
controller.servo_proportional(1, 1.0)
```

#### Drive Servos
Three drive modes are supported: ackerman, differential, and mecanum.

Specify the desired drive mode with the corresponding string value.
  
* __ackerman__ - A single velocity drive using one or more servos with a non-drive servo controlling steering.
* __differential__ - Skid steer or track steer using one or more servos for each of the left and right sides.
The result is full speed forward or reverse, min/max radius turns, and the ability to perform zero-radius turns.
* __mecanum__ - Independent drive on all four wheel capable of holonomic drive - moving in any combination of forward, reverse, turning, and sideways. This mode supports similar drive characteristics to differential drive with the additional of lateral motion.

The servos positions are assigned with `config_servo()`.

Servos drive parameters consists of the __drive mode__ (text name), __X__ direction value (-1.0 .. 1.0), __Y__ direction value (-1.0 .. 1.0), __rotation__ direction value (-1.0 .. 1.0) as a percentage of ±90 degrees. Note: the three values are mixed together and are limited to the maximum speed of each servo. For example, a value of X=1.0 and R=1.0 can not run both servos at 100% and still result in a 90 degree turn. You can see the results of the internal calculation when setting the Python logging level to `DEBUG`.


Servo proportional parameters consists of the __servo number__ (1 .. 16) and a __proportional value__ (-1.0 .. 1.0).


```
# drive robot 100% speed forward
controller.servos_drive("differential", 1.0, 0.0, 0.0)

# drive robot 33% speed backward
controller.servos_drive("differential", -0.33, 0.0, 0.0)

# drive robot in an arc to the right
controller.servos_drive("differential", 0.5, 0.0, 0.25)
```

#### Stop All Servos
Setting a servo to its absolute center value or its 0.0 proportional value has the effect or preventing the servo from moving.
This is often referred to as a 'brake'.
The stop command is used to set the servos to idle - also referred to as 'coast'.

```
# power down all servos
controller.stop_servos()
```

