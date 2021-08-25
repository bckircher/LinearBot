// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.utils.Log;
import frc.robot.utils.NavX;

/**
 * This command spins the robot to a face a given heading.
 */
public class TurnToAngle extends CommandBase {
  private final Drive m_drive;
  private final NavX m_navX;
  private final double m_angle;
  private final double m_maxSpeed;
  private final boolean m_hold;

  /**
   * This command spins the robot to face a given heading.
   *
   * <p>Once the heading is reached, the robot can be held at that heading (if
   * <i>hold</i> is <b>true</b>) or the command can finish without further
   * control of the robot (if <i>hold</i> is <b>false</b>). The former is
   * useful when the command is bound to a controller button, while the later
   * is useful when the command is utilized inside a command sequence.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param navX is the {@link NavX} sensor to use.
   *
   * @param angle is the heading to which the robot should be turned.
   *
   * @param maxSpeed is the maximum speed that the robot should travel.
   *
   * @param hold is whether the robot should be held at the given heading.
   */
  public TurnToAngle(Drive drive, NavX navX, double angle, double maxSpeed,
                     boolean hold) {
    m_drive = drive;
    m_navX = navX;
    m_angle = angle;
    m_maxSpeed = maxSpeed;
    m_hold = hold;

    addRequirements(m_drive);
  }

  /**
   * This command spins the robot to face a given heading.
   *
   * <p>Once the heading is reached, the command finishes.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param navX is the {@link NavX} sensor to use.
   *
   * @param angle is the heading to which the robot should be turned.
   *
   * @param maxSpeed is the maximum speed that the robot should travel.
   */
  public TurnToAngle(Drive drive, NavX navX, double angle, double maxSpeed) {
    this(drive, navX, angle, maxSpeed, false);
  }

  /**
   * This command spins the robot to face a given heading.
   *
   * <p>The robot is spun with a maximum speed of 0.5. Once the heading is
   * reached, the command finishes.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param navX is the {@link NavX} sensor to use.
   *
   * @param angle is the heading to which the robot should be turned.
   */
  public TurnToAngle(Drive drive, NavX navX, double angle) {
    this(drive, navX, angle, 0.5, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this, m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error, rotation;

    // Determine how far the robot is from the desired heading.
    error = m_angle - m_navX.getAngle();

    // Compute the amount that the robot should rotate.
    rotation = error * 0.1;

    // If the rotation rate is not zero but too small, increase it to the
    // minimum (so the robot has enough power to move).
    if((rotation > 0) && (rotation < 0.2)) {
      rotation = 0.2;
    }
    if((rotation < 0) && (rotation > -0.2)) {
      rotation = -0.2;
    }

    // If the rotation rate is greater than the maximum, reduce it to the
    // maximum.
    if(rotation > m_maxSpeed) {
      rotation = m_maxSpeed;
    }
    if(rotation < -m_maxSpeed) {
      rotation = -m_maxSpeed;
    }

    // Spin the robot at the computed rate.
    m_drive.driveArcade(0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_hold || (Math.abs(m_angle - m_navX.getAngle()) > 0.5)) {
      return false;
    }
    return true;
  }
}
