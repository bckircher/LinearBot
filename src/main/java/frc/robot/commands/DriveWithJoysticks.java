// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.utils.Log;

/**
 * This is intended to be used as the default command for the {@link Drive}
 * subsystem. It takes values from the given DoubleSuppliers (obstensibly a
 * stick on a controller) and uses their values to directly drive the robot.
 */
public class DriveWithJoysticks extends CommandBase {
  Drive m_drive;
  DoubleSupplier m_speed;
  DoubleSupplier m_rotation;

  /**
   * Creates an instance of this class.
   *
   * <p>This is intended to be used as the default command for the {@link
   * Drive} subsystem. It takes values from the given DoubleSuppliers
   * (obstensibly a stick on a controller) and uses their values to directly
   * drive the robot.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param speed is the DoubleSupplier used to query the speed and direction
   *              in which the robot should move.
   *
   * @param rotation is the DoubleSupplier used to query the amount that the
   *                 robot should rotate.
   */
  public DriveWithJoysticks(Drive drive, DoubleSupplier speed,
                            DoubleSupplier rotation) {
    m_drive = drive;
    m_speed = speed;
    m_rotation = rotation;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveArcade(m_speed.getAsDouble(), m_rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
    m_drive.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
