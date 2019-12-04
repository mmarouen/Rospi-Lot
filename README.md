# Rospi-Lot
Inspired by comma.ai OpenPilot idea, this is an AD Autopilot on RaspberryPi based on ROS called RosPilot. The project currently performs lane follow feature.
The codebase is written to be modular, enable quick prototyping and facilitate learning and collaboration across multiple users.
The hardware used so far is the donkey car robocar + RPI 3 

## Architecture

- Blocs represent named ros nodes.
- Arrows represent topics exchanged across ros master
- Boxes represent logical entities
![architecture](https://github.com/mmarouen/Rospi-Lot/blob/develop/images/architecture.png)

## Get it to run

1. Install ros on a raspberrypi, [ubiquity-robotics](https://downloads.ubiquityrobotics.com/pi.html) is a good opensource starting project
3. Connect your RPI to you PC and clone this repo under your workspace source folder **in the RPI** generally located under `~/catkin_ws/src`

## Further details
Please let me know <azzouz.marouen@gmail.com>
