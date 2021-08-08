// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.frc4048.Logging;

public class Elevator extends SubsystemBase {
  CANSparkMax m_left = new CANSparkMax(Constants.Elevator.kMotorLeft,
                                       MotorType.kBrushless);
  CANSparkMax m_right = new CANSparkMax(Constants.Elevator.kMotorRight,
                                        MotorType.kBrushless);

  CANEncoder m_leftPos = m_left.getEncoder();
  CANEncoder m_rightPos = m_right.getEncoder();

  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Left Elevator Applied Voltage", m_left.getAppliedOutput());
        add("Left Elevator Current", m_left.getOutputCurrent());
        add("Left Elevator Position", m_leftPos.getPosition());
        add("Right Elevator Applied Voltage", m_right.getAppliedOutput());
        add("Right Elevator Current", m_right.getOutputCurrent());
        add("Right Elevator Position", m_rightPos.getPosition());
      }
    };

  /** Creates a new Elevator. */
  public Elevator() {
    m_left.restoreFactoryDefaults();
    m_left.setIdleMode(IdleMode.kBrake);

    m_right.restoreFactoryDefaults();
    m_right.setIdleMode(IdleMode.kBrake);

    m_leftPos.setPosition(0);
    m_rightPos.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    double delta = m_leftPos.getPosition() - m_rightPos.getPosition();
  
    if(Math.abs(speed) > 0.5) {
      if(speed < 0) {
        speed = -0.5;
      } else {
        speed = 0.5;
      }
    }

    m_left.set(speed);
    if((speed == 0) && (Math.abs(delta) < 10)) {
      m_right.set(0);
    } else {
      m_right.set(speed + (delta * Constants.Elevator.kP));
    }
  }

  public void run(double left, double right) {
    m_left.set(left);
    m_right.set(right);
  }

  public void resetEncoders() {
    m_leftPos.setPosition(0);
    m_rightPos.setPosition(0);
  }
}
