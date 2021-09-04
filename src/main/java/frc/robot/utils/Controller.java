// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
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
public final class Controller {
  private static double m_deadBand = 0;

  /**
   * Applies a dead-band to the given value.<p>
   *
   * Values below the dead-band threshold are clamped to zero, while values
   * above the dead-band threshold are transformed to linearly range from 0 to
   * 1 as the input ranges between the dead-band threshold and 1.
   *
   * @param value is the value to dead-band.
   *
   * @return Returns the dead-banded version of the input value.
   */
  private static double applyDeadBand(double value) {
    if(Math.abs(value) < m_deadBand) {
      return 0;
    } else {
      return (value - m_deadBand) / (1.0 - m_deadBand);
    }
  }

  /**
   * Sets the dead-band threshold.<p>
   *
   * Values below the deadb-and threshold are clamped to zero, while values
   * above the dead-band threshold are transformed to linearly range from 0 to
   * 1 as the input ranges between the dead-band threshold and 1.<p>
   *
   * Note: The dead-band threshold applies equally to all the analog
   * controls of every controller.
   *
   * @param deadBand is the dead-band threshold to apply; must be between 0 (to
   *                 disable dead-banding) and 1 (all values are clamped to
   *                 zero).
   */
  public static void setDeadBand(double deadBand) {
    m_deadBand = deadBand;
  }

  /**
   * Gets the X value for the left stick of the given controller.<p>
   *
   * Dead-banding is applied to the value to keep small variations from zero,
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
    double value;

    // Get the X value of the left stick.
    value = controller.getRawAxis(Constants.Controller.kAnalogLeftX);
  
    // Apply the dead-band to the value and return it.
    return applyDeadBand(value);
  }

  /**
   * Gets the Y value for the left stick of the given controller.<p>
   *
   * Dead-banding is applied to the value to keep small variations from zero,
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
    double value;

    // Get the Y value of the left stick. The value is inverted since the stick
    // returns negative number when it is deflected up.
    value = -controller.getRawAxis(Constants.Controller.kAnalogLeftY);

    // Apply the dead-band to the value and return it.
    return applyDeadBand(value);
  }

  /**
   * Gets the X value for the right stick of the given controller.<p>
   *
   * Dead-banding is applied to the value to keep small variations from zero,
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
    double value;

    // Get the X value of the right stick.
    value = controller.getRawAxis(Constants.Controller.kAnalogRightX);

    // Apply the dead-band to the value and return it.
    return applyDeadBand(value);
  }

  /**
   * Gets the Y value for the right stick of the given controller.<p>
   *
   * Dead-banding is applied to the value to keep small variations from zero,
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
    double value;

    // Get the Y value of the right stick. The value is inverted since the
    // stick returns negative number when it is deflected up.
    value = -controller.getRawAxis(Constants.Controller.kAnalogRightY);

    // Apply to dead-band to the value and return it.
    return applyDeadBand(value);
  }

  /**
   * Gets the value for the L2 control of the given controller.<p>
   *
   * Dead-banding is applied to the value to keep small variations from zero,
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
    double value;

    // Get the value of the L2 control.
    value = controller.getRawAxis(Constants.Controller.kAnalogLeft2);

    // The value ranges from -1 (fully released) to 1 (fully pressed), so scale
    // it to range from 0 (fully released) to 1 (fully pressed), apply the
    // dead-band, and return the value.
    return applyDeadBand((value + 1) / 2);
  }

  /**
   * Gets the value for the R2 control of the given controller.<p>
   *
   * Dead-banding is applied to the value to keep small variations from zero,
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
    double value;

    // Get the value of the R2 control.
    value = controller.getRawAxis(Constants.Controller.kAnalogRight2);

    // The value ranges from -1 (fully released) to 1 (fully pressed), so scale
    // it to range from 0 (fully released) to 1 (fully pressed), apply the
    // dead-band, and return the value.
    return applyDeadBand((value + 1) / 2);
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
        return controller.getPOV() == angle;
      }
    };
  }
}