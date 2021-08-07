// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HDrive;
import frc.robot.utils.Logging;

/**
 * This is intended to be used as the default command for the HDrive subsystem.
 * It takes values from the given DoubleSupplier (obstensibly a stick on a
 * controller) and uses their values to directly drive the robot.
 */
public class RunHDrive extends CommandBase {
  HDrive m_hdrive;
  DoubleSupplier m_left;
  DoubleSupplier m_right;

  /** Creates a new RunHDrive. */
  /**
   * Creates an instance of this class.
   *
   * @param hdrive is the HDrive subsystem to use.
   *
   * @param left is the DoubleSupplier used to get the speed to move to the
   *             left.
   *
   * @param right is the DoubleSupplier used to get the speed to move to the
   *              right.
   */
  public RunHDrive(HDrive hdrive, DoubleSupplier left, DoubleSupplier right) {
    m_hdrive = hdrive;
    m_left = left;
    m_right = right;

    addRequirements(m_hdrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logging.logInit(this.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_left.getAsDouble();
    double right = m_right.getAsDouble();

    if((left == 0) && (right > 0)) {
      m_hdrive.run(right);
    } else if((right == 0) && (left > 0)) {
      m_hdrive.run(-left);
    } else {
      m_hdrive.run(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logging.logEnd(this.toString(), interrupted);
    m_hdrive.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
