// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Log;

/**
 * This command allows each side of the {@link Elevator} to be run manually,
 * allowing it to be put back into a level condition.
 */
public class TrimElevator extends CommandBase {
  private final Elevator m_elevator;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  /**
   * This command trims the elevator based on driver control.
   *
   * <p>This command allows each side of the {@link Elevator} to be run
   * manually, allowing it to be put back into a level condition.
   *
   * @param elevator is the {@link Elevator} subsystem to use.
   *
   * @param left is the DoubleSupplier used to query the speed and direction
   *             of the left side of the elevator.
   *
   * @param right is the DoubleSupplier used to query the speed and direction
   *              of the right side of the elevator.
   */
  public TrimElevator(Elevator elevator, DoubleSupplier left,
                      DoubleSupplier right) {
    m_elevator = elevator;
    m_left = left;
    m_right = right;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.run(m_left.getAsDouble(), m_right.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
    m_elevator.run(0, 0);
    m_elevator.resetPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}