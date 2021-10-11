// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.utils.NavX;

/**
 * This command spins the robot to a face a given heading.
 */
public class TurnToAngle extends CommandBase {
  /**
   * The default maximum speed to spin the robot, ranging from 0 to 1 (though
   * it really doesn't makes sense for it to be less that kS, so that the robot
   * will actually spin!).
   */
  private static final double m_defaultSpeed = 0.5;

  /**
   * The minimum speed to spin the robot, which is just large enough to get the
   * robot to move (overcoming static friction, hence kS).
   */
  private static final double m_kS = 0.225;

  /**
   * The proportional feedback constant for the speed controller.
   */
  private static final double m_kP = 0.0005;

  /**
   * The position error used to determine when the target has been reached.
   */
  private static final double m_kError = 0.5;

  /**
   * The {@link Drive} subsystem used to spin the robot.
   */
  private final Drive m_drive;

  /**
   * The {@link NavX} sensor that is used to monitor the robot's heading.
   */
  private final NavX m_navX;

  /**
   * The heading to which the robot should be turned.
   */
  private final double m_angle;

  /**
   * The maximum speed at which the robot should travel.
   */
  private final double m_maxSpeed;

  /**
   * If the command should "hold" the robot's rotation when it reaches the
   * target, or if it should finish (useful for use in command sequences).
   */
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
    this(drive, navX, angle, m_defaultSpeed, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error, rotation;

    // Determine how far the robot is from the desired heading.
    error = m_navX.getAngle() - m_angle;

    // Square the error (maintaining the sign). This makes the error
    // exponentially larger when it is far from the target but get increasingly
    // smaller as it approaches the target. This helps it to avoid
    // overshooting.
    error = Math.copySign(error * error, error);

    // Set the error to zero if it is close enough to the target angle.
    if(Math.abs(error) < m_kError) {
      error = 0;
    }

    // Compute the amount that the robot should rotate.
    rotation = error * m_kP;

    // If the rotation rate is not zero but too small, increase it to the
    // minimum (so the robot has enough power to move).
    if((rotation > 0) && (rotation < m_kS)) {
      rotation = m_kS;
    }
    if((rotation < 0) && (rotation > -m_kS)) {
      rotation = -m_kS;
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
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_hold || (Math.abs(m_navX.getAngle() - m_angle) > m_kError)) {
      return false;
    }
    return true;
  }
}
