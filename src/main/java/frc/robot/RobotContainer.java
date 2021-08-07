// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.utils.Controls;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive m_drive = new Drive();
  private final Elevator m_elevator = new Elevator();
  private final HDrive m_hdrive = new HDrive();
  private final Intake m_intake = new Intake();

  private final Joystick m_driverJoystick =
    new Joystick(Constants.Controller.kDriver);
  private final Joystick m_manipulatorJoystick =
    new Joystick(Constants.Controller.kManipulator);

  /**
   *  The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    Controls.setDeadband(Constants.Controller.kDeadband);

    m_drive.setDefaultCommand(
      new DriveWithJoysticks(m_drive,
                             () -> Controls.getLeftY(m_driverJoystick),
                             () -> Controls.getRightX(m_driverJoystick)));

    m_elevator.setDefaultCommand(
      new RunElevator(m_elevator,
                      () -> Controls.getLeftY(m_manipulatorJoystick)));

    m_hdrive.setDefaultCommand(
      new RunHDrive(m_hdrive,
                    () -> Controls.getLeft2(m_driverJoystick),
                    () -> Controls.getRight2(m_driverJoystick)));

    m_intake.setDefaultCommand(
      new RunIntake(m_intake,
                    () -> Controls.getLeft2(m_manipulatorJoystick),
                    () -> Controls.getRight2(m_manipulatorJoystick)));

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
                                 () -> Controls.
                                         getLeftX(m_manipulatorJoystick),
                                 () -> Controls.
                                         getRightX(m_manipulatorJoystick)));

    // Run a command while the driver's POV is set to 90 degrees.
    Controls.newPOVButton(m_driverJoystick, 90).
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
