// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunHDrive;
import frc.robot.commands.RunIntake;
import frc.robot.commands.TrimElevator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HDrive;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Controller;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel(11);
  private final Drive m_drive = new Drive();
  private final Elevator m_elevator = new Elevator();
  private final HDrive m_hdrive = new HDrive();
  private final Intake m_intake = new Intake(m_pdp);

  private final Joystick m_driverJoystick =
    new Joystick(Constants.Controller.kDriver);
  private final Joystick m_manipulatorJoystick =
    new Joystick(Constants.Controller.kManipulator);

  /**
   *  The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Set the deadband to apply to the analog controls.
    Controller.setDeadband(Constants.Controller.kDeadband);

    // Publish the state of the scheduler to SmartDashboard.
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Publish the state of the subsystems to SmartDashboard.
    SmartDashboard.putData(m_pdp);
    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_hdrive);
    SmartDashboard.putData(m_intake);

    // Add buttons to SmartDashboard to allow commands to be manually run.
    SmartDashboard.putData("Wait", new WaitCommand(1));

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
    m_hdrive.setDefaultCommand(
      new RunHDrive(m_hdrive,
                    () -> Controller.getLeft2(m_driverJoystick),
                    () -> Controller.getRight2(m_driverJoystick)));

    // Set the default command for the Intake subsystem.
    m_intake.setDefaultCommand(
      new RunIntake(m_intake,
                    () -> Controller.getLeft2(m_manipulatorJoystick),
                    () -> Controller.getRight2(m_manipulatorJoystick)));

    // Configure the controller buttons.
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Run a command while the driver's pink square button is held.
    new JoystickButton(m_driverJoystick,
                       Constants.Controller.kButtonPinkSquare).
      whileHeld(new WaitCommand(15));

    // Allow the elevator to be trimmed by the manipulator while the
    // manipulator L1 button is held.
    new JoystickButton(m_manipulatorJoystick,
                       Constants.Controller.kButtonLeft1).
      whileHeld(new TrimElevator(m_elevator,
                                 () -> Controller.
                                         getLeftY(m_manipulatorJoystick),
                                 () -> Controller.
                                         getRightY(m_manipulatorJoystick)));

    // Run a command while the driver's POV is set to 90 degrees.
    Controller.newPOVButton(m_driverJoystick, 90).
      whileHeld(new WaitCommand(15));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(15);
  }
}
