// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.utils.NavX;

/**
 * This command drives the robot a given distance, using the NavX gyro to
 * maintain its heading.
 */
public class DriveForDistance extends CommandBase {
  /**
   * The default maximum speed to drive the robot, ranging from 0 to 1 (though
   * it really doesn't makes sense for it to be less that kS, so that the robot
   * will actually move!).
   */
  private static final double m_defaultSpeed = 0.5;

  /**
   * The minimum speed to drive the robot, which is just large enough to get
   * the robot to move (overcoming static friction, hence kS).
   */
  private static final double m_kS = 0.2;

  /**
   * The proportional feedback constant for the speed controller.
   */
  private static final double m_kP_Speed = 1;

  /**
   * The proportional feedback constant for the steering controller.
   */
  private static final double m_kP_Steer = 0.05;

  /**
   * The position error used to determine when the target has been reached.
   */
  private static final double m_kError_Position = 0.01;

  /**
   * The heading error used to determine final steering
   */
  private static final double m_kError_Heading = 0.5;

  /**
   * The {@link Drive} subsystem that is used to move the robot.
   */
  private final Drive m_drive;

  /**
   * The {@link NavX} sensor that is used to monitor the robot's heading.
   */
  private final NavX m_navX;

  /**
   * The distance that the robot should travel.
   */
  private final double m_distance;

  /**
   * The maximum speed at which the robot should travel.
   */
  private final double m_maxSpeed;

  /**
   * If the command should "hold" the robot's position when it reaches the
   * target, or if it should finish (useful for use in command sequences).
   */
  private final boolean m_hold;

  /**
   * The direction the robot was facing when the command was started, which is
   * therefore the direction of travel it tries to maintain.
   */
  private double m_direction;

  /**
   * The target position for the robot.
   */
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
    this(drive, navX, distance, m_defaultSpeed, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    if(Math.abs(error) < m_kError_Position) {
      speed = 0;
    } else {
      speed = error * m_kP_Speed;
    }

    // If the speed is not zero but too small, increase it to the minimum (so
    // the robot has enough power to move).
    if((speed != 0) && (Math.abs(speed) < m_kS)) {
      if(speed > 0) {
        speed = m_kS;
      } else {
        speed = -m_kS;
      }
    }

    // If the speed is greater than the maximum, reduce it to the maximum.
    if(Math.abs(speed) > m_maxSpeed) {
      if(speed > 0) {
        speed = m_maxSpeed;
      } else {
        speed = -m_maxSpeed;
      }
    }

    // Determine how far the robot is from the target direction.
    error = m_navX.getAngle() - m_direction;

    // Compute the rotation that the robot should turn to get back to going
    // the desired direction.
    if((speed == 0) && (Math.abs(error) < m_kError_Heading)) {
      rotation = 0;
    } else {
      rotation = error * m_kP_Steer;
      if((Math.abs(speed) + Math.abs(rotation)) < m_kS) {
        rotation = Math.copySign(m_kS - Math.abs(speed) - Math.abs(rotation),
                                 rotation);
      }
    }

    // Move the robot at the computed speed.
    m_drive.driveArcade(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_hold ||
       (Math.abs(m_target - m_drive.getPosition()) > m_kError_Position) ||
       (Math.abs(m_navX.getAngle() - m_direction) > m_kError_Heading)) {
      return false;
    }
    return true;
  }
}
