// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.frc4048.Logging;

public class Elevator extends SubsystemBase {
  private final CANSparkMax m_left =
    new CANSparkMax(Constants.Elevator.kMotorLeft, MotorType.kBrushless);
  private final CANSparkMax m_right =
    new CANSparkMax(Constants.Elevator.kMotorRight, MotorType.kBrushless);

  private final CANEncoder m_leftEncoder = m_left.getEncoder();
  private final CANEncoder m_rightEncoder = m_right.getEncoder();

  private double m_leftTemp = 0;
  private double m_rightTemp = 0;

  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Left Applied Voltage", m_left.getAppliedOutput());
        add("Left Current", m_left.getOutputCurrent());
        add("Left Temperature", m_left.getMotorTemperature());
        add("Left SPARK MAX Stick Faults", m_left.getStickyFaults());
        add("Left Position", m_leftEncoder.getPosition());
        add("Right Applied Voltage", m_right.getAppliedOutput());
        add("Right Current", m_right.getOutputCurrent());
        add("Right Temperature", m_right.getMotorTemperature());
        add("Right SPARK MAX Stick Faults", m_right.getStickyFaults());
        add("Right Position", m_rightEncoder.getPosition());
      }
    };

  /** Creates a new Elevator. */
  public Elevator() {
    m_left.restoreFactoryDefaults();
    m_left.setOpenLoopRampRate(1);
    m_left.setIdleMode(IdleMode.kBrake);

    m_right.restoreFactoryDefaults();
    m_right.setOpenLoopRampRate(1);
    m_right.setIdleMode(IdleMode.kBrake);

    m_leftEncoder.setPositionConversionFactor(Constants.Elevator.
                                                kEncoderConversion);
    m_rightEncoder.setPositionConversionFactor(Constants.Elevator.
                                                 kEncoderConversion);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Temp: Left", () -> m_leftTemp, null);
    builder.addBooleanProperty("Temp: LeftGood",
        () -> m_leftTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Left Position",
                              () -> m_leftEncoder.getPosition(), null);
    builder.addDoubleProperty("Temp: Right", () -> m_rightTemp, null);
    builder.addBooleanProperty("Temp:: RightGood",
        () -> m_rightTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Right Position",
                              () -> m_rightEncoder.getPosition(), null);
  }

  @Override
  public void periodic() {
    // Update the motor temperature when the count matches the motor ID.
    if(Robot.getPeriodicCount() == Constants.Elevator.kMotorLeft) {
      m_leftTemp = m_left.getMotorTemperature();
    }
    if(Robot.getPeriodicCount() == Constants.Elevator.kMotorRight) {
      m_rightTemp = m_right.getMotorTemperature();
    }
  }

  public void run(double speed) {
    double delta = m_leftEncoder.getPosition() - m_rightEncoder.getPosition();
  
    if(Math.abs(speed) > 0.5) {
      if(speed < 0) {
        speed = -0.5;
      } else {
        speed = 0.5;
      }
    }

    m_left.set(speed);
    if((speed == 0) && (Math.abs(delta) < 0.01)) {
      m_right.set(0);
    } else {
      m_right.set(speed + (delta * Constants.Elevator.kP));
    }
  }

  public void run(double left, double right) {
    m_left.set(left / 4);
    m_right.set(right / 4);
  }

  public void stop() {
    m_left.stopMotor();
    m_right.stopMotor();
  }

  public void resetPosition() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getPosition() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
  }
}