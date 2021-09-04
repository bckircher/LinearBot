// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HDrive;
import frc.robot.utils.Log;

/**
 * This command strafes the robot a given distance.
 *
 * <p>Strafing happens by first running the H-drive slowly, to place the
 * appropriate wheel onto the ground then driving the requested distance from
 * that wheel position (since driving at that point will actually cause the
 * robot to move across the ground).
 */
public class StrafeForDistance extends CommandBase {
  private static final double m_defaultSpeed = 0.5;
  private static final double m_kS_Initial = 0.1;
  private static final double m_kS = 0.2;
  private static final double m_kP = 0.2;
  private static final double m_kError = 0.01;
  private final HDrive m_hDrive;
  private final double m_distance;
  private final double m_maxSpeed;
  private final boolean m_hold;
  private double m_target;
  private boolean m_ready;

  /**
   * This command strafes the robot a given distance.
   *
   * <p>Once the distance is reached, the robot can be held at that target (if
   * <i>hold</i> is <b>true</b>) or the command can finish without further
   * control of the robot (if <i>hold</i> is <b>false</b>). The former is
   * useful when the command is bound to a controller button, while the later
   * is useful when the command is utilized inside a command sequence.
   *
   * @param hDrive is the {@link HDrive} subsystem to use.
   *
   * @param distance is the distance the robot should travel.
   *
   * @param maxSpeed is the maximum speed that the robot should travel.
   *
   * @param hold is whether the robot should be held at the given distance.
   */
  public StrafeForDistance(HDrive hDrive, double distance, double maxSpeed,
                           boolean hold) {
    m_hDrive = hDrive;
    m_distance = distance;
    m_maxSpeed = maxSpeed;
    m_hold = hold;

    addRequirements(m_hDrive);
  }

  /**
   * This command strafes the robot a given distance.
   *
   * <p>Once the distance is reached, the command finishes.
   *
   * @param hDrive is the {@link HDrive} subsystem to use.
   *
   * @param distance is the distance the robot should travel.
   *
   * @param maxSpeed is the maximum speed that the robot should travel.
   */
  public StrafeForDistance(HDrive hDrive, double distance, double maxSpeed) {
    this(hDrive, distance, maxSpeed, false);
  }

  /**
   * This command strafes the robot a given distance.
   *
   * <p>Once the distance is reached, the command finishes.
   *
   * @param hDrive is the {@link HDrive} subsystem to use.
   *
   * @param distance is the distance the robot should travel.
   */
  public StrafeForDistance(HDrive hDrive, double distance) {
    this(hDrive, distance, m_defaultSpeed, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this, m_distance);

    // The robot is not ready to strafe; the wheel must be put onto the ground
    // first.
    m_ready = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error, speed;

    // See if the robot is ready to strafe (in other words, the wheel is on the
    // ground).
    if(m_ready == false) {
      // If the current is spiking high enough, it indicates that the wheel has
      // reached the ground.
      if(m_hDrive.getCurrent() > 30) {
        // Set the target as the requested distance from the current position
        // of the strafe motor.
        m_target = m_hDrive.getPosition() + m_distance;

        // The robot is ready to strafe.
        m_ready = true;
      } else {
        // Drive the motor slowly in the requested direction to place the wheel
        // onto the ground.
        m_hDrive.run(Math.copySign(m_kS_Initial, m_distance));
      }
    } else {
      // Determine how far the robot is from the target position.
      error = m_target - m_hDrive.getPosition();

      // Compute the speed that the robot should move.
      speed = error * m_kP;

      // If the speed is not zero but too small, increase it to the minimum (so
      // the robot has enough power to move).
      if((speed != 0) && (speed < m_kS)) {
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

      // Do not allow the H-Drive to move in the opposite direction.
      if(((m_distance < 0) && (speed > 0)) ||
         ((m_distance > 0) && (speed < 0))) {
        speed = 0;
      }

      // Strafe the robot at the computed speed.
      m_hDrive.run(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_hold || !m_ready ||
       (Math.abs(m_target - m_hDrive.getPosition()) > m_kError)) {
      return false;
    }
    return true;
  }
}