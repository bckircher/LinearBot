// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Logging;

/**
 * This is intended to be used as the default command for the Elevator
 * subsystem. It takes values from the given DoubleSupplier (obstensibly a
 * stick on a controller) and uses its value to directly move the elevator.
 */
public class RunElevator extends CommandBase {
  Elevator m_elevator;
  DoubleSupplier m_speed;

  /**
   * Creates an instance of this class.
   * @param elevator is the Elevator subsystem to use.
   * @param speed is the DoubleSupplier used to query the speed and direction
   * to move the elevator.
   */
  public RunElevator(Elevator elevator, DoubleSupplier speed) {
    m_elevator = elevator;
    m_speed = speed;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logging.logInit(this.getClass().getSimpleName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.run(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logging.logEnd(this.getClass().getSimpleName(), interrupted);
    m_elevator.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
