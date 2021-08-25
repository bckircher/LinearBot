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
 * This is intended to be used as the default command for the {@link Elevator}
 * subsystem. It takes values from the given DoubleSupplier (ostensibly a stick
 * on a controller) and uses its value to directly move the elevator.
 */
public class RunElevator extends CommandBase {
  private final Elevator m_elevator;
  private final DoubleSupplier m_speed;

  /**
   * This command runs the elevator based on driver control.
   *
   * <p>This is intended to be used as the default command for the {@link
   * Elevator} subsystem. It takes values from the given DoubleSupplier
   * (ostensibly a stick on a controller) and uses its value to directly move
   * the elevator.
   *
   * @param elevator is the {@link Elevator} subsystem to use.
   *
   * @param speed is the DoubleSupplier used to query the speed and direction
   *              to move the elevator.
   */
  public RunElevator(Elevator elevator, DoubleSupplier speed) {
    m_elevator = elevator;
    m_speed = speed;

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
    m_elevator.run(m_speed.getAsDouble() / 2.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
