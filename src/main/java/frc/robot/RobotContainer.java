// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ElevatorToHeight;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunHDrive;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TrimElevator;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HDrive;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Controller;
import frc.robot.utils.Log;
import frc.robot.utils.NavX;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * The {@link NavX} object used by the robot.
   */
  private final NavX m_navX = new NavX();

  /**
   * The {@link PowerDistributionPanel} object used by the robot.
   */
  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();

  /**
   * The {@link Drive} subsystem used by the robot.
   */
  private final Drive m_drive = Drive.create(m_navX);

  /**
   * The {@link Elevator} subsystem used by the robot.
   */
  private final Elevator m_elevator = Elevator.create();

  /**
   * The {@link HDrive} subsystem used by the robot.
   */
  private final HDrive m_hDrive = HDrive.create();

  /**
   * The {@link Intake} subsystem used by the robot.
   */
  private final Intake m_intake = Intake.create(m_pdp);

  /**
   * The controller used by the driver.
   */
  private final Joystick m_driverJoystick =
    new Joystick(Constants.Controller.kDriver);

  /**
   * The controller used by the manipulator.
   */
  private final Joystick m_manipulatorJoystick =
    new Joystick(Constants.Controller.kManipulator);

  /**
   * The first PathWeaver trajectory.
   */
  private final Trajectory m_trajectory1;

  /**
   * The second PathWeaver trajectory.
   */
  private final Trajectory m_trajectory2;

  private final Trajectory m_trajectory3;

  /**
   * The camera connected to the USB port of the roboRIO.
   */
  private final UsbCamera m_camera;

  /**
   * The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Load the autonomous trajectories.
    m_trajectory1 = loadTrajectory("Test1.wpilib.json");
    m_trajectory2 = loadTrajectory("Test2.wpilib.json");
    m_trajectory3 = loadTrajectory("Test3.wpilib.json");

    // Set the dead-band to apply to the analog controls.
    Controller.setDeadBand(Constants.Controller.kDeadBand);

    // Publish various items to SmartDashboard.
    publishToSmartDashboard();

    // Configure the default commands for each subsystem.
    configureDefaultCommands();

    // Configure the controller buttons.
    configureButtonBindings();

    // Start streaming the first camera to the driver station.
    m_camera = CameraServer.getInstance().startAutomaticCapture(0);
    m_camera.setResolution(640, 480);

    // Use the scheduler to log the scheduling and execution of commands.
    CommandScheduler.getInstance().
      onCommandInitialize(command -> Log.init(command));
    CommandScheduler.getInstance().
      onCommandInterrupt(command -> Log.end(command, true));
    CommandScheduler.getInstance().
      onCommandFinish(command -> Log.end(command, false));
  }

  /**
   * Load a PathWeaver trajectory from the deploy directory.
   */
  private Trajectory loadTrajectory(String json) {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().
                              resolve("output/" + json);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + json,
                                ex.getStackTrace());
      return null;
    }
    return trajectory;
  }

  /**
   * Publishes items to SmartDashboard for use by the drivers/programmers.
   */
  private void publishToSmartDashboard() {
    // Publish the state of the scheduler to SmartDashboard.
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Publish the state of the subsystems to SmartDashboard.
    SmartDashboard.putData(m_pdp);
    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_hDrive);
    SmartDashboard.putData(m_intake);

    // Add buttons to SmartDashboard to allow commands to be manually run.
    SmartDashboard.putData("Drive 1'",
                           new DriveForDistance(m_drive, m_navX,
                                                Units.feetToMeters(1)));
    SmartDashboard.putData("Drive 5'",
                           new DriveForDistance(m_drive, m_navX,
                                                Units.feetToMeters(5)));
    SmartDashboard.putData("Elevator 0'", new ElevatorToHeight(m_elevator, 0));
    SmartDashboard.putData("Elevator 1'",
                           new ElevatorToHeight(m_elevator,
                                                Units.feetToMeters(1)));
    SmartDashboard.putData("Elevator 2'",
                           new ElevatorToHeight(m_elevator,
                                                Units.feetToMeters(2)));
    SmartDashboard.putData("Elevator 3'",
                           new ElevatorToHeight(m_elevator,
                                                Units.feetToMeters(3)));
    SmartDashboard.putData("Trajectory 1",
                           new FollowTrajectory(m_drive, m_trajectory1));
    SmartDashboard.putData("Trajectory 2",
                           new FollowTrajectory(m_drive, m_trajectory2));

    SmartDashboard.putData("Reset H",
                           new RunCommand(() -> m_hDrive.resetPosition()));
  }

  /**
   * Sets the default commands for each of the subsystems.
   */
  private void configureDefaultCommands() {
    // Set the default command for the Drive subsystem.
    m_drive.setDefaultCommand(
      new DriveWithJoysticks(m_drive,
                             () -> Controller.getLeftY(m_driverJoystick),
                             () -> Controller.getRightX(m_driverJoystick)));

    // Set the default command for the Elevator subsystem.
    m_elevator.setDefaultCommand(
      new RunElevator(m_elevator,
                      () -> Controller.getLeftY(m_manipulatorJoystick)));

    // Set the default command for the HDrive subsystem.
    m_hDrive.setDefaultCommand(
      new RunHDrive(m_hDrive,
                    () -> Controller.getLeft2(m_driverJoystick),
                    () -> Controller.getRight2(m_driverJoystick)));

    // Set the default command for the Intake subsystem.
    m_intake.setDefaultCommand(
      new RunIntake(m_intake,
                    () -> Controller.getLeft2(m_manipulatorJoystick),
                    () -> Controller.getRight2(m_manipulatorJoystick)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Follow the first trajectory while the driver's pink square is held.
    new JoystickButton(m_driverJoystick,
                       Constants.Controller.kButtonPinkSquare).
      whileHeld(new FollowTrajectory(m_drive, m_trajectory1).andThen(new WaitCommand(60)));

    // Follow the second trajectory while the driver's red circle is held.
    new JoystickButton(m_driverJoystick,
                       Constants.Controller.kButtonRedCircle).
      whileHeld(new FollowTrajectory(m_drive, m_trajectory2).andThen(new WaitCommand(60)));

    new JoystickButton(m_driverJoystick,
                      Constants.Controller.kButtonGreenTriangle).
      whileHeld(new FollowTrajectory(m_drive, m_trajectory3).andThen(new WaitCommand(60)));

    // Turn the robot to 0 degrees when the driver's POV is set to 0 degrees
    // (in other words, up).
    Controller.newPOVButton(m_driverJoystick, 0).
      whileHeld(new TurnToAngle(m_drive, m_navX, 0, 0.5, true));

    // Turn the robot to 90 degrees when the driver's POV is set to 90 degrees
    // (in other words, right).
    Controller.newPOVButton(m_driverJoystick, 90).
      whileHeld(new TurnToAngle(m_drive, m_navX, 90, 0.5, true));

    // Turn the robot to 180 degrees when the driver's POV is set to 180
    // degrees (in other words, down).
    Controller.newPOVButton(m_driverJoystick, 180).
      whileHeld(new TurnToAngle(m_drive, m_navX, 180, 0.5, true));

    // Turn the robot to 270 degrees when the driver's POV is set to 270
    // degrees (in other words, down).
    Controller.newPOVButton(m_driverJoystick, 270).
      whileHeld(new TurnToAngle(m_drive, m_navX, 270, 0.5, true));

    // Move the elevator to 30" when the manipulator's POV is set to 0 degrees
    // (in other words, up).
    Controller.newPOVButton(m_manipulatorJoystick, 0).
      whileHeld(new ElevatorToHeight(m_elevator, Units.inchesToMeters(30), 0.5,
                                     true));

    // Move the elevator to 20" when the manipulator's POV is set to 90 degrees
    // (in other words, to the right).
    Controller.newPOVButton(m_manipulatorJoystick, 90).
      whileHeld(new ElevatorToHeight(m_elevator, Units.inchesToMeters(20), 0.5,
                                     true));

    // Move the elevator to 10" when the manipulator's POV is set to 180
    // degrees (in other words, down).
    Controller.newPOVButton(m_manipulatorJoystick, 180).
      whileHeld(new ElevatorToHeight(m_elevator, Units.inchesToMeters(10), 0.5,
                                     true));

    // Move the elevator to 0" when the manipulator's POV is set to 270 degrees
    // (in other words, to the left).
    Controller.newPOVButton(m_manipulatorJoystick, 270).
      whileHeld(new ElevatorToHeight(m_elevator, 0, 0.5, true));

    // Allow the elevator to be trimmed by the manipulator while the
    // manipulator L1 button is held.
    new JoystickButton(m_manipulatorJoystick,
                       Constants.Controller.kButtonLeft1).
      whileHeld(new TrimElevator(m_elevator,
                                 () -> Controller.
                                         getLeftY(m_manipulatorJoystick),
                                 () -> Controller.
                                         getRightY(m_manipulatorJoystick)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
