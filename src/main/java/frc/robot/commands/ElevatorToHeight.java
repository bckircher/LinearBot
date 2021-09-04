// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Log;

/**
 * This command moves the elevator to a specific height.
 */
public class ElevatorToHeight extends CommandBase {
  private static final double m_defaultSpeed = 0.5;
  private static final double m_kS = 0.1;
  private static final double m_kP = 0.1;
  private static final double m_kError = 0.01;
  private final Elevator m_elevator;
  private final double m_target;
  private final double m_maxSpeed;
  private final boolean m_hold;

  /**
   * This command moves the elevator to a specific height.
   *
   * <p>Once the target is reached, the elevator can be held at that target (if
   * <i>hold</i> is <b>true</b>) or the command can finish without further
   * control of the elevator (if <i>hold</i> is <b>false</b>). The former is
   * useful when the command is bound to a controller button, while the later
   * is useful when the command is utilized inside a command sequence.
   *
   * @param elevator is the {@link Elevator} subsystem to use.
   *
   * @param target is the height to which the elevator should be moved.
   *
   * @param maxSpeed is the maximum speed the elevator should move.
   *
   * @param hold is whether the elevator should be held at the given height.
   */
  public ElevatorToHeight(Elevator elevator, double target, double maxSpeed,
                          boolean hold) {
    m_elevator = elevator;
    m_target = target;
    m_maxSpeed = maxSpeed;
    m_hold = hold;

    addRequirements(m_elevator);
  }

  /**
   * This command moves the elevator to a specific height.
   *
   * <p>Once the target is reached, the command finishes.
   *
   * @param elevator is the {@link Elevator} subsystem to use.
   *
   * @param target is the height to which the elevator should be moved.
   *
   * @param maxSpeed is the maximum speed the elevator should move.
   */
  public ElevatorToHeight(Elevator elevator, double target, double maxSpeed) {
    this(elevator, target, maxSpeed, false);
  }

  /**
   * This command moves the elevator to a specific height.
   *
   * <p>The elevator is moved with a maximum speed of 0.5. Once the target is
   * reached, the command finishes.
   *
   * @param elevator is the {@link Elevator} subsystem to use.
   *
   * @param target is the height to which the elevator should be moved.
   */
  public ElevatorToHeight(Elevator elevator, double target) {
    this(elevator, target, m_defaultSpeed, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this, m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error, speed;

    // Determine how far the elevator is away from the desired height.
    error = m_target - m_elevator.getPosition();

    // Compute the speed that the elevator should move.
    speed = error * m_kP;

    // If the speed is not zero but too small, increase it to the minimum (so
    // the elevator has enough power to move).
    if((speed > 0) && (speed < m_kS)) {
      speed = m_kS;
    }
    if((speed < 0) && (speed > -m_kS)) {
      speed = -m_kS;
    }

    // If the speed is greater than the maximum, reduce it to the maximum.
    if(speed > m_maxSpeed) {
      speed = m_maxSpeed;
    }
    if(speed < -m_maxSpeed) {
      speed = -m_maxSpeed;
    }

    // Move the elevator at the computed speed.
    m_elevator.run(speed);
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
    if(m_hold || (Math.abs(m_target - m_elevator.getPosition()) > m_kError)) {
      return false;
    }
    return true;
  }
}