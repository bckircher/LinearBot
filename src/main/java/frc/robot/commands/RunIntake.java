// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Log;

/**
 * This is intended to be used as the default command for the {@link Intake}
 * subsystem. It takes values from the given DoubleSuppliers (ostensibly analog
 * buttons on a controller) and uses their values to directly control the
 * intake.
 */
public class RunIntake extends CommandBase {
  /**
   * The {@link Intake} subsystem to use.
   */
  private final Intake m_intake;

  /**
   * The DoubleSupplier that determines how much to run the intake in the in
   * direction.
   */
  private final DoubleSupplier m_in;

  /**
   * The DoubleSupplier that determines how much to run the intake in the out
   * direction.
   */
  private final DoubleSupplier m_out;

  /**
   * This command runs the intake based on driver control.
   *
   * <p>This is intended to be used as the default command for the {@link
   * Intake}subsystem. It takes values from the given DoubleSuppliers
   * (ostensibly analog buttons on a controller) and uses their values to
   * directly control the intake.
   *
   * <p>The intake subsystem is rather unique. Running the motor one direction
   * will pull in cargo (if the robot is not in possession of a game piece) but
   * eject a hatch panel (if the robot is in possession of a hatch panel),
   * while running the motor the other direction will eject cargo (if the robot
   * is in possession of cargo) but intake a hatch panel (if the robot is not
   * in possession of a game piece). For the purposes of this class, in versus
   * out is defined with respect to cargo; "inward" means to pull in cargo
   * while "outward" means to eject cargo.
   *
   * @param intake is the {@link Intake} subsystem to use.
   *
   * @param in is the DoubleSupplier used to query the speed at which to run
   *           the intake in an "inward" direction.
   *
   * @param out is the DoubleSupplier used to query the speed at which to run
   *            the intake in an "outward" direction.
   */
  public RunIntake(Intake intake, DoubleSupplier in, DoubleSupplier out) {
    m_intake = intake;
    m_in = in;
    m_out = out;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Log.init(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double in = m_in.getAsDouble();
    double out = m_out.getAsDouble();

    // Run the intake in or out, based on the control that is being deflected.
    // If both are being deflected, do not move.
    if((in == 0.0) && (out > 0.0)) {
      m_intake.run(out);
    } else if((out == 0.0) && (in > 0.0)) {
      m_intake.run(-in);
    } else {
      m_intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Log.end(this, interrupted);
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
