// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

/**
 * This class provides methods for getting values from the controllers,
 * utilizing some signal conditioning to clean up the results.
 */
public final class Controls {
  private static double m_deadband = 0;

  /**
   * Applies a deadband to the given value.<p>
   *
   * Values below the deadband threshold are clamped to zero, while values
   * above the deadband threshold are transformed to linearly range from 0 to
   * 1 as the input ranges between the deadband threshold and 1.
   *
   * @param value is the value to deadband.
   *
   * @return Returns the deadbanded version of the input value.
   */
  private static double applyDeadband(double value) {
    if(Math.abs(value) < m_deadband) {
      return(0);
    } else {
      return((value - m_deadband) / (1.0 - m_deadband));
    }
  }

  /**
   * Sets the deadband threashold.<p>
   *
   * Values below the deadband threshold are clamped to zero, while values
   * above the deadband threshold are transformed to linearly range from 0 to
   * 1 as the input ranges between the deadband threshold and 1.<p>
   *
   * Note: The deadband threshold applies equally to all the analog
   * controls of every controller.
   *
   * @param deadband is the deadband threshold to apply; must be between 0 (to
   *                 disable deadbanding) and 1 (all values are clamped to
   *                 zero).
   */
  public static void setDeadband(double deadband) {
    m_deadband = deadband;
  }

  /**
   * Gets the X value for the left stick of the given controller.<p>
   *
   * Deadbanding is applied to the value to keep small variations from zero,
   * when the stick is released, from imparting a movement command into the
   * system. Positive values indicate that the stick is deflected to the right
   * and negative values indicate that the stick is deflected to the left.
   *
   * @param controller is the controller to query.
   *
   * @return Returns the X value of the controller's left stick, ranging from
   *         -1 to 1.
   */
  public static double getLeftX(Joystick controller) {
    return(applyDeadband(controller.getRawAxis(Constants.Controller.
                                               kAnalogLeftX)));
  }

  /**
   * Gets the Y value for the left stick of the given controller.<p>
   *
   * Deadbanding is applied to the value to keep small variations from zero,
   * when the stick is released, from imparting a movement command into the
   * system. Positive values indicate that the stick is deflected up and
   * negative values indicate that the stick is deflected down.
   *
   * @param controller is the controller to query.
   *
   * @return Returns the Y value of the controller's left stick, ranging from
   *         -1 to 1.
   */
  public static double getLeftY(Joystick controller) {
    return(applyDeadband(controller.getRawAxis(Constants.Controller.
                                               kAnalogLeftY)));
  }

  /**
   * Gets the X value for the right stick of the given controller.<p>
   *
   * Deadbanding is applied to the value to keep small variations from zero,
   * when the stick is released, from imparting a movement command into the
   * system. Positive values indicate that the stick is deflected to the right
   * and negative values indicate that the stick is deflected to the left.
   *
   * @param controller is the controller to query.
   *
   * @return Returns the X value of the controller's right stick, ranging from
   *         -1 to 1.
   */
  public static double getRightX(Joystick controller) {
    return(applyDeadband(controller.getRawAxis(Constants.Controller.
                                               kAnalogRightX)));
  }

  /**
   * Gets the Y value for the right stick of the given controller.<p>
   *
   * Deadbanding is applied to the value to keep small variations from zero,
   * when the stick is released, from imparting a movement command into the
   * system. Positive values indicate that the stick is deflected up and
   * negative values indicate that the stick is deflected down.
   *
   * @param controller is the controller to query.
   *
   * @return Returns the Y value of the controller's right stick, ranging from
   *         -1 to 1.
   */
  public static double getRightY(Joystick controller) {
    return(applyDeadband(controller.getRawAxis(Constants.Controller.
                                               kAnalogRightY)));
  }

  /**
   * Gets the value for the L2 control of the given controller.<p>
   *
   * Deadbanding is applied to the value to keep small variations from zero,
   * when the stick is released, from imparting a movement command into the
   * system. 0 indicates that the control is fully released and 1 indicates
   * that the control is fully pressed.
   *
   * @param controller is the controller to query.
   *
   * @return Returns the value of the controller's L2 control, ranging from 0
   *         to 1.
   */
  public static double getLeft2(Joystick controller) {
    return(applyDeadband((controller.getRawAxis(Constants.Controller.
                                                kAnalogLeft2) + 1) / 2));
  }

  /**
   * Gets the value for the R2 control of the given controller.<p>
   *
   * Deadbanding is applied to the value to keep small variations from zero,
   * when the stick is released, from imparting a movement command into the
   * system. 0 indicates that the control is fully released and 1 indicates
   * that the control is fully pressed.
   *
   * @param controller is the controller to query.
   *
   * @return Returns the value of the controller's R2 control, ranging from 0
   *         to 1.
   */
  public static double getRight2(Joystick controller) {
    return(applyDeadband((controller.getRawAxis(Constants.Controller.
                                                kAnalogRight2) + 1) / 2));
  }

  /**
   * Creates a virtual button that is pressed whenever the POV controller is
   * at a particular angle.
   *
   * @param controller is the controller to query.
   *
   * @param angle is the angle at which the virtual button is considered to be
   *              pressed.
   *
   * @return Returns a JoystickButton that is pressed when the POV controller
   *         is at a particular angle.
   */
  public static JoystickButton newPOVButton(Joystick controller, int angle) {
    return new JoystickButton(controller, angle) {
      @Override
      public boolean get() {
        return(controller.getPOV() == angle);
      }
    };
  }
}
