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
    public final static int kButtonPinkSquare = 0;

    /**
     * The index of the blue X button on the controller.
     */
    public final static int kButtonBlueX = 1;

    /**
     * The index of the red circle button on the controller.
     */
    public final static int kButtonRedCircle = 2;

    /**
     * The index of the green triangle button on the controller.
     */
    public final static int kButtonGreenTriangle = 3;

    /**
     * The index of the L1 button on the controller.
     */
    public final static int kButtonLeft1 = 4;

    /**
     * The index of the R1 button on the controller.
     */
    public final static int kButtonRight1 = 5;

    /**
     * The index of the L2 button on the controller.
     */
    public final static int kButtonLeft2 = 6;

    /**
     * The index of the R2 button on the controller.
     */
    public final static int kButtonRight2 = 7;

    /**
     * The index of the share button on the controller.
     */
    public final static int kButtonShare = 8;

    /**
     * The index of the options button on the controller.
     */
    public final static int kButtonOptions = 9;

    /**
     * The index of the L3 button (pressing down on the left stick) on the
     * controller.
     */
    public final static int kButtonLeft3 = 10;

    /**
     * The index of the R3 button (pressing down on the right stick) on the
     * controller.
     */
    public final static int kButtonRight3 = 11;

    /**
     * The index of the home button on the controller.
     */
    public final static int kButtonHome = 12;

    /**
     * The index of the touchpad button on the controller.
     */
    public final static int kButtonTouchpad = 13;

    /**
     * The amount of deadband to apply to the analog controls.
     */
    public static final double kDeadband = 0.05;
  }
}
