Copyright (c) 2021 FRC Team 2881 - The Lady Cans

Open Source Software; you can modify and/or share it under the terms of BSD
license file in the root directory of this project.


# Overview

This project contains the code to drive the 2019/2020/2021 linear elevator
robot that was created in the off-season.


# Hardware Mapping

These are the CAN IDs of the devices on the robot:

| CAN ID | Device                             |
|--------|------------------------------------|
| 0      | Power Distribution Panel (PDP)     |
| 1      | Front left drive motor (SparkMax)  |
| 2      | Back left drive motor (SparkMax)   |
| 3      | Front right drive motor (SparkMax) |
| 4      | Back right drive motor (SparkMax)  |
| 5      | Left elevator motor (SparkMax)     |
| 6      | Right elevator motor (SparkMax)    |
| 7      | H-drive motor (SparkMax)           |

These are the PWM channels of the devices on the robot:

| PWM Channel | Device               |
|-------------|----------------------|
| 0           | Intake motor (Spark) |

These are the PDP channels for the devices on the robot:

| PDP Channel | Device                  |
|-------------|-------------------------|
| 0           | Elevator right motor    |
| 1           | H-Drive motor           |
| 2           | Elevator left motor     |
| 7           | Intake motor            |
| 12          | Right front drive motor |
| 13          | Right back drive motor  |
| 14          | Left front drive motor  |
| 15          | Left back drive motor   |


# Controls

## Driver

* Left joystick Y and right joystick X -> split arcade drive

* L2/R2 -> strafe left and right

## Manipulator

* L2/R2 -> intake, where L2 will ingest cargo and R2 will eject cargo (with
  hatch panels being handled by swapping the controls)

* Left joystick Y -> elevator up/down

* While L1 is held -> left joystick Y controls the left side of the elevator
                      and right joystick Y controls the right side of the
                      elevator for manual leveling; the encoders are reset when
                      L1 is released, setting the code's idea of level and its
                      idea of the bottom of travel.


# ToDo

* [X] Check Drive subsystem odometry/pose
* [X] Test Elevator subsystem
* [X] Tune Elevator subsystem
* [ ] Tune ElevatorToHeight command
* [?] Characterize drive train
* [ ] Test FollowTrajectory command
* [X] Tune Intake subsystem
* [ ] Unit tests