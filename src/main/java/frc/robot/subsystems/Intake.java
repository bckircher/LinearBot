// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.frc4048.Logging;

/**
 * This subsystem controls the intake.
 */
public class Intake extends SubsystemBase {
  /**
   * The {@link PowerDistributionPanel} that is used to determine the current
   * flowing through the intake motor.
   */
  private final PowerDistributionPanel m_pdp;

  /**
   * The motor that drives the intake.
   */
  private final Spark m_intake;

  /**
   * The target speed of the intake.
   */
  private double m_target = 0;

  /**
   * The current speed of the intake.
   */
  private double m_speed = 0;

  /**
   * The current flowing through the intake motor.
   */
  private double m_current = 0;

  private final boolean m_useLoop = true;

  /**
   * The logging context for this subsystem, which writes the relevant
   * information about the state of the subsystem to a CSV file for later
   * analysis.
   */
  /*
  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Applied Voltage", m_intake.getSpeed());
        add("Current", m_current);
      }
    };
  */

  /**
   * Creates a new Intake.
   *
   * @param pdp the {@link PowerDistributionPanel} to use to measure the
   *            current flowing through the intake motor.
   */
  public Intake(PowerDistributionPanel pdp, Spark intake) {
    // Save the subsystem objects.
    m_pdp = pdp;
    m_intake = intake;
  }

  /**
   * Creates a new Intake object.
   *
   * @param pdp the {@link PowerDistributionPanel} to use to measure the
   *            current flowing through the intake motor.
   *
   * @return The new Intake object.
   */
  public static Intake create(PowerDistributionPanel pdp) {
    // Create the Spark motor controller for the intake motor.
    Spark motor = new Spark(Constants.Intake.kMotor);

    // Create the Intake object.
    Intake intake = new Intake(pdp, motor);

    // Add periodic logging for the new Intake object.
    new Logging.LoggingContext(intake.getClass()) {
      @Override
      protected void addAll() {
        add("Applied Voltage", intake.m_intake.getSpeed());
        add("Current", intake.m_current);
      }
    };

    // Return the new Intake object.
    return intake;
  }

  // Adds items from this subsystem to the sendable builder, so that they get
  // sent to the driver station via network tables.
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Intake target", () -> m_target, null);
    builder.addDoubleProperty("Intake speed", () -> m_speed, null);
  }

  // This method is called every 20 ms.
  @Override
  public void periodic() {
    double overCurrent;

    // Get the current flowing through the intake motor.
    m_current = m_pdp.getCurrent(Constants.PDP.kIntake);
  
    // There is nothing to do if the target and current speeds are both zero.
    if((m_target == 0) && (m_speed == 0)) {
      return;
    }

    // Determine if the intake motor is consuming too much current, and if so
    // by how much.
    overCurrent = m_current - Constants.Intake.kMaxCurrent;
    if(overCurrent < 0) {
      overCurrent = 0;
    }

    // Compute the desired speed of the motor, slowing it down if it is
    // consuming too much current. Limit the speed reduction to zero, not
    // allowing the motor to run in the opposite direction (the expression
    // checks for the signs of the target and speed not matching).
    m_speed = (m_target -
               Math.copySign(overCurrent * Constants.Intake.kP, m_target));
    if((m_target * m_speed) < 0) {
      m_speed = 0;
    }

    // Apply the computed speed to the intake motor.
    if(m_useLoop) {
      m_intake.set(m_speed);
    }
  }

  /**
   * Runs the intake.
   *
   * @param speed the speed to run the intake.
   */
  public void run(double speed) {
    m_target = speed;
    if(!m_useLoop) {
      m_intake.set(speed);
    }
  }

  /**
   * Stops the intake motor.
   */
  public void stop() {
    m_target = 0;
    if(!m_useLoop) {
      m_intake.stopMotor();
    }
  }
}