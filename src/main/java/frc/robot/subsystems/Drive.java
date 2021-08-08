// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.frc4048.Logging;

public class Drive extends SubsystemBase {
  CANSparkMax m_leftFront = new CANSparkMax(Constants.Drive.kMotorLeftFront,
                                            MotorType.kBrushless);
  CANSparkMax m_leftBack = new CANSparkMax(Constants.Drive.kMotorLeftBack,
                                           MotorType.kBrushless);
  CANSparkMax m_rightFront = new CANSparkMax(Constants.Drive.kMotorRightFront,
                                             MotorType.kBrushless);
  CANSparkMax m_rightBack = new CANSparkMax(Constants.Drive.kMotorRightBack,
                                            MotorType.kBrushless);

  DifferentialDrive m_drive = new DifferentialDrive(m_leftFront, m_rightFront);

  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Left Front Applied Voltage", m_leftFront.getAppliedOutput());
        add("Left Front Current", m_leftFront.getOutputCurrent());
        add("Left Back Applied Voltage", m_leftBack.getAppliedOutput());
        add("Left Back Current", m_leftBack.getOutputCurrent());
        add("Right Front Applied Voltage", m_rightFront.getAppliedOutput());
        add("Right Front Current", m_rightFront.getOutputCurrent());
        add("Right Back Applied Voltage", m_rightBack.getAppliedOutput());
        add("Right Back Current", m_rightBack.getOutputCurrent());
      }
    };

  /** Creates a new Drive. */
  public Drive() {
    m_leftFront.restoreFactoryDefaults();
    m_leftFront.setIdleMode(IdleMode.kBrake);

    m_leftBack.restoreFactoryDefaults();
    m_leftBack.setIdleMode(IdleMode.kBrake);

    m_leftBack.follow(m_leftFront);

    m_rightFront.restoreFactoryDefaults();
    m_rightFront.setIdleMode(IdleMode.kBrake);

    m_rightBack.restoreFactoryDefaults();
    m_rightBack.setIdleMode(IdleMode.kBrake);
  
    m_rightBack.follow(m_rightFront);

    m_drive.setSafetyEnabled(true);
    m_drive.setExpiration(0.1);

    addChild("Differential Drive", m_drive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveArcade(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation, true);
  }

  public void driveTank(double left, double right) {
    m_drive.tankDrive(left, right, true);
  }
}
