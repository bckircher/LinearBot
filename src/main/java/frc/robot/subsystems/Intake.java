// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.frc4048.Logging;

public class Intake extends SubsystemBase {
  private final Spark m_intake = new Spark(Constants.Intake.kMotor);
  private final PowerDistributionPanel m_pdp;
  private double m_target = 0;
  private double m_speed = 0;
  private double m_current = 0;
  private final boolean m_useLoop = false;

  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Applied Voltage", m_intake.getSpeed());
        add("Current", m_current);
      }
    };

  /** Creates a new Intake. */
  public Intake(PowerDistributionPanel pdp) {
    m_pdp = pdp;

    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Intake target", () -> m_target, null);
    builder.addDoubleProperty("Intake speed", () -> m_speed, null);
  }

  @Override
  public void periodic() {
    if(false) {
    double error, speed;

    m_current = m_pdp.getCurrent(Constants.PDP.kIntake);
  
    // There is nothing to do if the target and current speeds are both zero.
    if((m_target == 0) && (m_speed == 0)) {
      return;
    }

    error = Constants.Intake.kMaxCurrent - m_current;

    speed = m_speed + Math.copySign(error * Constants.Intake.kP, m_target);
    if(Math.abs(speed) > Math.abs(m_target)) {
      speed = m_target;
    }

    m_speed = speed;

    if(m_useLoop) {
      m_intake.set(m_speed);
    }
    }
  }

  public void run(double speed) {
    m_target = speed;
    if(!m_useLoop) {
      m_intake.set(speed);
    }
  }

  public void stop() {
    m_target = 0;
    if(!m_useLoop) {
      m_intake.stopMotor();
    }
  }
}