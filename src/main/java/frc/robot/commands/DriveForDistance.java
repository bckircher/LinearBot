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
 * This command drives the robot a given distance, using the NavX gyro to
 * maintain its heading.
 */
public class DriveForDistance extends CommandBase {
  private final Drive m_drive;
  private final NavX m_navX;
  private final double m_distance;
  private final double m_maxSpeed;
  private final boolean m_hold;
  private double m_direction;
  private double m_target;

  /**
   * This command drives the robot a given distance, using the NavX gyro to
   * maintain its heading.
   *
   * <p>Once the distance is reached, the robot can be held at that target (if
   * <i>hold</i> is <b>true</b>) or the command can finish without further
   * control of the robot (if <i>hold</i> is <b>false</b>). The former is
   * useful when the command is bound to a controller button, while the later
   * is useful when the command is utilized inside a command sequence.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param navX is the (@link NavX} sensor to use.
   *
   * @param distance is the distance the robot should travel.
   *
   * @param maxSpeed is the maximum speed that the robot should travel.
   *
   * @param hold is whether the robot should be held at the given distance.
   */
  public DriveForDistance(Drive drive, NavX navX, double distance,
                          double maxSpeed, boolean hold) {
    m_drive = drive;
    m_navX = navX;
    m_distance = distance;
    m_maxSpeed = maxSpeed;
    m_hold = hold;
  
    addRequirements(m_drive);
  }

  /**
   * This command drives the robot a given distance, using the NavX gyro to
   * maintain its heading.
   *
   * <p>Once the distance is reached, the command finishes.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param navX is the (@link NavX} sensor to use.
   *
   * @param distance is the distance the robot should travel.
   *
   * @param maxSpeed is the maximum speed that the robot should travel.
   */
  public DriveForDistance(Drive drive, NavX navX, double distance,
                          double maxSpeed) {
    this(drive, navX, distance, maxSpeed, false);
  }

  /**
   * This command drives the robot a given distance, using the NavX gyro to
   * maintain its heading.
   *
   * <p>The robot is driven with a maximum speed of 0.5. Once the distance is
   * reached, the command finishes.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param navX is the (@link NavX} sensor to use.
   *
   * @param distance is the distance the robot should travel.
   */
  public DriveForDistance(Drive drive, NavX navX, double distance) {
    this(drive, navX, distance, 0.5, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this, m_distance);

    m_direction = m_navX.getAngle();

    m_target = m_drive.getPosition() + m_distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error, speed, rotation;

    // Determine how far the robot is from the target position.
    error = m_target - m_drive.getPosition();

    // Compute the speed that the robot should move.
    speed = error * 0.1;

    // If the speed is not zero but too small, increase it to the minimum (so
    // the robot has enough power to move).
    if((speed > 0) && (speed < 0.2)) {
      speed = 0.2;
    }
    if((speed < 0) && (speed > -0.2)) {
      speed = -0.2;
    }

    // If the speed is greater than the maximum, reduce it to the maximum.
    if(speed > m_maxSpeed) {
      speed = m_maxSpeed;
    }
    if(speed < -m_maxSpeed) {
      speed = -m_maxSpeed;
    }

    // Determine how far the robot is from the target direction.
    error = m_direction - m_navX.getAngle();

    // Compute the rotation that the robot should turn to get back to going
    // the desired direction.
    rotation = error * 0.1;

    // Move the robot at the computed speed.
    m_drive.driveArcade(speed, rotation);
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
    if(m_hold || (Math.abs(m_target - m_drive.getPosition()) > 0.1)) {
      return false;
    }
    return true;
  }
}
