// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Constants {
  /**
   * The channel allocations on the PDP (Power Distribution Panel).
   */
  public static final class PDP {
    /**
     * The PDP channel that powers the front left motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveLeftFront = 1;
    
    /**
     * The PDP channel that powers the back left motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveLeftBack = 2;

    /**
     * The PDP channel that powers the front right motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveRightFront = 3;

    /**
     * The PDP channel that powers the back right motor of the {@link Drive}
     * subsystem.
     */
    public static final int kDriveRightBack = 4;

    /**
     * The PDP channel that powers the left motor of the {@link Elevator}
     * subsystem.
     */
    public static final int kElevatorLeft = 5;

    /**
     * The PDP channel that powers the right motor of the {@link Elevator}
     * subsystem.
     */
    public static final int kElevatorRight = 6;

    /**
     * The PDP channel that powers the motor of the {@link HDrive} subsystem.
     */
    public static final int kHDrive = 7;

    /**
     * The PDP channel that powers the motor of the {@link Intake} subsystem.
     */
    public static final int kIntake = 8;
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
  }

  /**
   * The constants for the HDrive subsysstem.
   */
  public static final class HDrive {
    /**
     * The CAN ID of the SparkMAX controller that drives the H-drive wheel.
     */
    public static final int kMotor = 7;
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
    public static final double kMaxCurrent = 10.0;
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
     * The index of the touchpad button on the controller.
     */
    public final static int kButtonTouchpad = 14;

    /**
     * The amount of deadband to apply to the analog controls.
     */
    public static final double kDeadband = 0.05;
  }
}
