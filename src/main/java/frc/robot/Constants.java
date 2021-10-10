// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utils.GearRatio;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Constants {
  /**
   * The maximum motor temperature allowed before the warning light is turned
   * on.
   */
  public static final double maxMotorTemperature = 40.0;

  /**
   * The channel allocations on the PDP (Power Distribution Panel).
   */
  public static final class PDP {
    /**
     * The PDP channel that powers the front left motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveLeftFront = 14;

    /**
     * The PDP channel that powers the back left motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveLeftBack = 15;

    /**
     * The PDP channel that powers the front right motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveRightFront = 12;

    /**
     * The PDP channel that powers the back right motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveRightBack = 13;

    /**
     * The PDP channel that powers the left motor of the {@link Elevator}
     * subsystem.
     */
    public static final int kElevatorLeft = 2;

    /**
     * The PDP channel that powers the right motor of the {@link Elevator}
     * subsystem.
     */
    public static final int kElevatorRight = 0;

    /**
     * The PDP channel that powers the motor of the {@link HDrive} subsystem.
     */
    public static final int kHDrive = 1;

    /**
     * The PDP channel that powers the motor of the {@link Intake} subsystem.
     */
    public static final int kIntake = 7;
  }

  /**
   * The constants for the Drive subsystem.
   */
  public static final class Drive {
    /**
     * The CAN ID of the SparkMAX controller that drives the front left wheel.
     */
    public static final int kMotorLeftFront = 1;

    /**
     * The CAN ID of the SparkMAX controller that drives the back left wheel.
     */
    public static final int kMotorLeftBack = 2;

    /**
     * The CAN ID of the SparkMAX controller that drives the front right wheel.
     */
    public static final int kMotorRightFront = 3;

    /**
     * The CAN ID of hte SparkMAX controller that drives the back right wheel.
     */
    public static final int kMotorRightBack = 4;

    /**
     * The conversion factor from encoder rotations to distance traveled by the
     * robot, in meters.
     */
    public static final double kEncoderConversion =
      GearRatio.computeWheel(1, 12, 50, 18, 36, Units.inchesToMeters(6));

    /**
     * The track width of the robot, in meters.
     */
    public static final double kTrackWidth = Units.inchesToMeters(26);

    /**
     * The kinematics object to convert chassis velocity into wheel speeds.
     */
    public static final DifferentialDriveKinematics kKinematics =
      new DifferentialDriveKinematics(kTrackWidth);

    /**
     * The amount of power to required to overcome static friction in the drive
     * train. Determined via frc-characterization.
     */
    public static final double kS = 0.176;

    /**
     * The amount of power required to maintain a specific velocity. Determined
     * via frc-characterization.
     */
    public static final double kV = 1.07;

    /**
     * The amount of power required to achieve a specific acceleration.
     * Determined via frc-characterization.
     */
    public static final double kA = 0.124;

    /**
     * The proportional constant for the feedback controller. Determined via
     * frc-characterization.
     */
    public static final double kP = 2.51;

    /**
     * The derivative constants for the feedback controller. Determined via
     * frc-characterization.
     */
    public static final double kD = 0;

    /**
     * The Ramsete trajectory follower Beta constant. This is a reasonable
     * baseline value.
     */
    public static final double kRamseteB = 2.0;

    /**
     * The Ramsete trajectory follower Zeta constant. This is a reasonable
     * baseline value.
     */
    public static final double kRamseteZeta = 0.7;
  }

  /**
   * The constants for the Elevator subsystem.
   */
  public static final class Elevator {
    /**
     * The CAN ID of the SparkMAX controller that drives the left side of the
     * elevator.
     */
    public static final int kMotorLeft = 5;

    /**
     * The CAN ID of the SparkMAX controller that drives the right side of the
     * elevator.
     */
    public static final int kMotorRight = 6;

    /**
     * The conversion factor from encoder rotations to distance traveled for
     * the elevator.
     */
    public static final double kEncoderConversion =
      GearRatio.computeLeadScrew(1, 16, 32, 2, 1, Units.inchesToMeters(0.5));

    /**
     * The maximum speed at which the elevator can be moved.
     */
    public static final double kMaxSpeed = 0.5;

    /**
     * The kP value for the P-controller used by the right side of the elevator
     * to track the left side of the elevator.
     */
    public static final double kP = 8.0;

    /**
     * The maximum allowable error in position between the left and right side
     * of the elevator.
     */
    public static final double kError = 0.01;
  }

  /**
   * The constants for the HDrive subsystem.
   */
  public static final class HDrive {
    /**
     * The CAN ID of the SparkMAX controller that drives the H-drive wheel.
     */
    public static final int kMotor = 7;

    /**
     * The conversion factor from encoder rotations to distance traveled by the
     * robot, in meters.
     */
    public static final double kEncoderConversion =
      GearRatio.computeWheel(1, 12, 50, 50, 64, Units.inchesToMeters(4));
  }

  /**
   * The constants for the Intake subsystem.
   */
  public static final class Intake {
    /**
     * The PWM channel of the Spark controller that drives the intake.
     */
    public static final int kMotor = 0;

    /**
     * The current limit on the intake motor.
     */
    public static final double kMaxCurrent = 8.0;

    /**
     * The kP value for the P-controller used by the current control loop.
     */
    public static final double kP = 0.05;
  }

  /**
   * The constants for resources on the driver controllers.
   */
  public static final class Controller {
    /**
     * The controller address of the game pad that is used for driver controls.
     */
    public final static int kDriver = 0;

    /**
     * The controller address of the game pad that is used for manipulator
     * controls.
     */
    public final static int kManipulator = 1;

    /**
     * The analog channel that returns data for the X position of the left
     * stick.
     */
    public final static int kAnalogLeftX = 0;

    /**
     * The analog channel that returns data for the Y position of the left
     * stick.
     */
    public final static int kAnalogLeftY = 1;

    /**
     * The analog channel that returns data for the X position of the right
     * stick.
     */
    public final static int kAnalogRightX = 2;

    /**
     * The analog channel that returns data for the amount the L2 button is
     * pressed.
     */
    public final static int kAnalogLeft2 = 3;

    /**
     * The analog channel that returns data for the amount the R2 button is
     * pressed.
     */
    public final static int kAnalogRight2 = 4;

    /**
     * The analog channel that returns data for the Y position of the right
     * stick.
     */
    public final static int kAnalogRightY = 5;

    /**
     * The index of the pink square button on the controller.
     */
    public final static int kButtonPinkSquare = 1;

    /**
     * The index of the blue X button on the controller.
     */
    public final static int kButtonBlueX = 2;

    /**
     * The index of the red circle button on the controller.
     */
    public final static int kButtonRedCircle = 3;

    /**
     * The index of the green triangle button on the controller.
     */
    public final static int kButtonGreenTriangle = 4;

    /**
     * The index of the L1 button on the controller.
     */
    public final static int kButtonLeft1 = 5;

    /**
     * The index of the R1 button on the controller.
     */
    public final static int kButtonRight1 = 6;

    /**
     * The index of the L2 button on the controller.
     */
    public final static int kButtonLeft2 = 7;

    /**
     * The index of the R2 button on the controller.
     */
    public final static int kButtonRight2 = 8;

    /**
     * The index of the share button on the controller.
     */
    public final static int kButtonShare = 9;

    /**
     * The index of the options button on the controller.
     */
    public final static int kButtonOptions = 10;

    /**
     * The index of the L3 button (pressing down on the left stick) on the
     * controller.
     */
    public final static int kButtonLeft3 = 11;

    /**
     * The index of the R3 button (pressing down on the right stick) on the
     * controller.
     */
    public final static int kButtonRight3 = 12;

    /**
     * The index of the home button on the controller.
     */
    public final static int kButtonHome = 13;

    /**
     * The index of the touch pad button on the controller.
     */
    public final static int kButtonTouchPad = 14;

    /**
     * The amount of dead-band to apply to the analog controls.
     */
    public static final double kDeadBand = 0.1;
  }
}
