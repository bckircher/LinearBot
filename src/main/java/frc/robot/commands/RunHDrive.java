// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HDrive;
import frc.robot.utils.Log;

/**
 * This is intended to be used as the default command for the {@link HDrive}
 * subsystem. It takes values from the given DoubleSupplier (ostensibly a stick
 * on a controller) and uses their values to directly drive the robot.
 */
public class RunHDrive extends CommandBase {
  private final HDrive m_hDrive;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  /**
   * This command runs the HDrive based on driver control.
   *
   * <p>This is intended to be used as the default command for the {@link
   * HDrive} subsystem. It takes values from the given DoubleSupplier
   * (ostensibly a stick on a controller) and uses their values to directly
   * drive the robot.
   *
   * @param hDrive is the {@link HDrive} subsystem to use.
   *
   * @param left is the DoubleSupplier used to get the speed to move to the
   *             left.
   *
   * @param right is the DoubleSupplier used to get the speed to move to the
   *              right.
   */
  public RunHDrive(HDrive hDrive, DoubleSupplier left, DoubleSupplier right) {
    m_hDrive = hDrive;
    m_left = left;
    m_right = right;

    addRequirements(m_hDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_left.getAsDouble();
    double right = m_right.getAsDouble();

    if((left == 0) && (right > 0)) {
      m_hDrive.run(right);
    } else if((right == 0) && (left > 0)) {
      m_hDrive.run(-left);
    } else {
      m_hDrive.run(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
    m_hDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
