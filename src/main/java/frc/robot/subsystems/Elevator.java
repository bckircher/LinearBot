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

public class Elevator extends SubsystemBase {
  CANSparkMax m_left = new CANSparkMax(Constants.Elevator.kMotorLeft,
                                       MotorType.kBrushless);
  CANSparkMax m_right = new CANSparkMax(Constants.Elevator.kMotorRight,
                                        MotorType.kBrushless);

  CANEncoder m_leftPos = m_left.getEncoder();
  CANEncoder m_rightPos = m_right.getEncoder();

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
    double delta = m_rightPos.getPosition() - m_leftPos.getPosition();
  
    m_left.set(speed);
    if((speed == 0) && (Math.abs(delta) < 10)) {
      m_right.set(0);
    } else {
      m_right.set(speed + (delta * 0.01));
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
