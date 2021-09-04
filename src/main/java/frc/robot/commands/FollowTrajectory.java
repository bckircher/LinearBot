// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.utils.Log;

/**
 * This command drives the robot along a given trajectory.
 */
public class FollowTrajectory extends CommandBase {
  private final RamseteCommand m_ramsete;

  /**
   * This command drives the robot along a given trajectory.
   *
   * <p>The provided trajectory is followed, after which the command finishes.
   * Trajectories can be manually constructed in code or loaded from a JSON
   * trajectory created using PathWeaver.
   *
   * @param drive is the {@link Drive} subsystem to use.
   *
   * @param trajectory is the trajectory to follow.
   */
  public FollowTrajectory(Drive drive, Trajectory trajectory) {
    // Reset the drive odometry to the initial pose of the trajectory.
    drive.resetOdometry(trajectory.getInitialPose());

    // The Ramsete controller for following the trajectory.
    RamseteController controller =
      new RamseteController(Constants.Drive.kRamseteB,
                            Constants.Drive.kRamseteZeta);

    // The helper to compute feed forward values for the motors.
    SimpleMotorFeedforward forward =
      new SimpleMotorFeedforward(Constants.Drive.kS,
                                 Constants.Drive.kV,
                                 Constants.Drive.kA);

    // The PID controller for the left and right motors.
    PIDController leftPID =
      new PIDController(Constants.Drive.kP, 0, Constants.Drive.kD);
    PIDController rightPID =
      new PIDController(Constants.Drive.kP, 0, Constants.Drive.kD);

    // The Ramsete command for following the trajectory.
    m_ramsete = new RamseteCommand(trajectory, drive::getPose, controller,
                                   forward, Constants.Drive.kKinematics,
                                   drive::getWheelSpeeds, leftPID, rightPID,
                                   drive::driveTankVolts, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this);
    m_ramsete.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ramsete.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
    m_ramsete.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ramsete.isFinished();
  }
}