# Public-2019-Robot-Project
Public release of ArmStrong's 2019 robot project.
The code is written in Java and uses WPILib's "Command based programming" design pattern.

## Key Features
- State based operation of robot mechanisms - arms are coordinated and synchronized to not collide with each other.
- Automatic alignment to vision targets using Pixy - custom self implemented algorithems to calculate robot position (x, y, theta) relative to the targets.
- Autonomous paths are generated and executed using custom algorithems and Jaci's Pathfinder.
- Calibration modes - Custom calibration modes for each robot mechanism that disable normal robot operation, use special controller button mapping and open Shuffleboard tabs with all the relevant data.

## Setup
The project is compatible with Visual Studio Code.
It requires WPILib 2019.4.1 and the following Vendor Libraries:
- KauaiLabs_navX_FRC 3.1.366
- PathfinderOld 2019.2.19
- CTRE-Phoenix 5.14.1
- REVRobotics 1.1.9
- [PixyUSB](https://github.com/Cybersonics/PixyUSB "PixyUSB GitHub Repository")
