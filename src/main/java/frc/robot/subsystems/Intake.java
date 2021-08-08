// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.frc4048.Logging;

public class Intake extends SubsystemBase {
  private Spark m_intake = new Spark(Constants.Intake.kMotor);
  private PowerDistributionPanel m_pdp;
  private double m_target;
  private double m_speed;
  private boolean m_useLoop = false;

  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Intake Applied Voltage", m_intake.getSpeed());
        add("Intake Current", m_pdp.getCurrent(Constants.PDP.kIntake));
        add("Intake Target", m_target);
        add("Intake Speed", m_speed);
      }
    };

  /** Creates a new Intake. */
  public Intake(PowerDistributionPanel pdp) {
    m_pdp = pdp;
    m_target = 0;
    m_speed = 0;

    SmartDashboard.putNumber("Intake target", 0);
    SmartDashboard.putNumber("Intake speed", 0);
  }

  @Override
  public void periodic() {
    double error, speed;
  
    // There is nothing to do if the target and current speeds are both zero.
    if((m_target == 0) && (m_speed == 0)) {
      return;
    }

    error = (Constants.Intake.kMaxCurrent -
             m_pdp.getCurrent(Constants.PDP.kIntake));

    speed = m_speed + Math.copySign(error * Constants.Intake.kP, m_target);
    if(Math.abs(speed) > Math.abs(m_target)) {
      speed = m_target;
    }

    SmartDashboard.putNumber("Intake speed", speed);
    m_speed = speed;

    if(m_useLoop) {
      m_intake.set(m_speed);
    }
  }

  public void run(double speed) {
    SmartDashboard.putNumber("Intake target", speed);
    m_target = speed;
    if(!m_useLoop) {
      m_intake.set(speed);
    }
  }
}
