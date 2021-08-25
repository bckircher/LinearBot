// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.frc4048.Logging;

public class HDrive extends SubsystemBase {
  private final CANSparkMax m_drive =
    new CANSparkMax(Constants.HDrive.kMotor, MotorType.kBrushless);

  private double m_driveTemp = 0;

  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Applied Voltage", m_drive.getAppliedOutput());
        add("Current", m_drive.getOutputCurrent());
        add("Temperature", m_drive.getMotorTemperature());
        add("SPARK MAX Sticky Faults", m_drive.getStickyFaults());
      }
    };

  /** Creates a new HDrive. */
  public HDrive() {
    m_drive.restoreFactoryDefaults();
    m_drive.setIdleMode(IdleMode.kCoast);

    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Temp: HDrive", () -> m_driveTemp, null);
    builder.addBooleanProperty("Temp:: HDriveGood",
        () -> m_driveTemp < Constants.maxMotorTemperature, null);
  }

  @Override
  public void periodic() {
    // Update the motor temperature when the count matches the motor ID.
    if(Robot.getPeriodicCount() == Constants.HDrive.kMotor) {
      m_driveTemp = m_drive.getMotorTemperature();
    }
  }

  public void run(double speed) {
    m_drive.set(-speed);
  }

  public void stop() {
    m_drive.stopMotor();
  }
}
