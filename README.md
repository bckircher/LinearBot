Copyright (c) 2021 FRC Team 2881 - The Lady Cans

Open Source Software; you can modify and/or share it under the terms of BSD
license file in the root directory of this project.

# Overview

This project contains the code to drive the 2019/2020/2021 linear elevator
'bot that was created in the off-season.

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
                      L1 is released, setting the code's idea of level.

# ToDo

* [X] Tune DriveForDistance command
* [X} Add gyro to DriveForDistance command
* [X] Tune StrafeForDistance command
* [ ] Tune Elevator subsystem
* [X] Tune TurnToAngle command
* [ ] Tune ElevatorToHeight command
* [ ] Characterize drive train
* [ ] Test FollowTrajectory command
* [X] Upgrade PDP firmware
* [X] Determine what's wrong with the PDP
* [ ] Tune Intake subsystem