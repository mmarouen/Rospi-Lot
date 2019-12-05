# Rospi-ADkit
Inspired by comma.ai OpenPilot idea, this is an AD Autopilot on RaspberryPi based on ROS called RosPilot. The project currently performs lane follow feature.
The codebase is written to be modular, enable quick prototyping and facilitate learning and collaboration across multiple users.
The hardware used so far is the donkey car robocar + RPI 3 

## Architecture

- Green blocks represent named ros nodes.
- Arrows represent topics exchanged across ros master
- Blue boxes represent logical entities

![architecture](https://github.com/mmarouen/Rospi-Lot/blob/develop/images/architecture.png)

## Get it to run

 1. Install ros on a raspberrypi, [ubiquity-robotics](https://downloads.ubiquityrobotics.com/pi.html) is a good opensource starting project
 2. Connect your RPI to you PC and clone this repo under your workspace source folder **in the RPI** generally located under `~/catkin_ws/src`
 3. Source the workspace and launch
 `cd catkin_ws`
 `source devel/setup.bash`
 4. Launch the vehicle
 `roslaunch frames_logger launcher_320x420.launch`

## Further details
Please let me know <azzouz.marouen@gmail.com>