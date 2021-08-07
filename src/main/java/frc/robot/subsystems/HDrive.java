// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HDrive extends SubsystemBase {
  CANSparkMax m_drive = new CANSparkMax(Constants.HDrive.kMotor,
                                        MotorType.kBrushless);

  /** Creates a new HDrive. */
  public HDrive() {
    m_drive.restoreFactoryDefaults();
    m_drive.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    m_drive.set(speed);
  }
}
