// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.frc4048.Logging;

/**
 * This subsystem controls the H-drive.
 */
public class HDrive extends SubsystemBase {
  /**
   * The motor that drives the H-drive wheels.
   */
  private final CANSparkMax m_drive;

  /**
   * The encoder that tracks the position of the H-drive wheels.
   */
  private final CANEncoder m_encoder;

  /**
   * The temperature of the H-drive motor.
   */
  private double m_driveTemp = 0;

  /**
   * The logging context for this subsystem, which writes the relevant
   * information about the state of the subsystem to a CSV file for later
   * analysis.
   */
  /*
  public Logging.LoggingContext loggingContext =
    new Logging.LoggingContext(this.getClass()) {
      @Override
      protected void addAll() {
        add("Applied Voltage", m_drive.getAppliedOutput());
        add("Current", m_drive.getOutputCurrent());
        add("Temperature", m_drive.getMotorTemperature());
        add("SPARK MAX Sticky Faults", m_drive.getStickyFaults());
      }
    };
  */

  /**
   * Creates a new HDrive.
   *
   * @param drive the motor controller for the HDrive.
   *
   * @param encoder the encoder that tracks the position of the HDrive.
   */
  public HDrive(CANSparkMax drive, CANEncoder encoder) {
    // Save the subsystem objects.
    m_drive = drive;
    m_encoder = encoder;

    // Configure the motor controller.
    m_drive.restoreFactoryDefaults();
    m_drive.setIdleMode(IdleMode.kCoast);

    // Set the conversion factor for the encoder so that it returns the
    // position in meters.
    m_encoder.setPositionConversionFactor(Constants.HDrive.kEncoderConversion);
  }

  /**
   * Creates a new HDrive object.
   *
   * @return The new HDrive object.
   */
  public static HDrive create() {
    // Create the SparkMax controller and encoder for use by the HDrive.
    CANSparkMax drive =
      new CANSparkMax(Constants.HDrive.kMotor, MotorType.kBrushless);
    CANEncoder encoder = drive.getEncoder();

    // Create the HDrive object.
    HDrive hDrive = new HDrive(drive, encoder);

    // Add periodic logging for the new HDrive object.
    new Logging.LoggingContext(hDrive.getClass()) {
      @Override
      protected void addAll() {
        add("Applied Voltage", hDrive.m_drive.getAppliedOutput());
        add("Current", hDrive.m_drive.getOutputCurrent());
        add("Temperature", hDrive.m_drive.getMotorTemperature());
        add("SPARK MAX Sticky Faults", hDrive.m_drive.getStickyFaults());
      }
    };

    // Return the new HDrive object.
    return hDrive;
  }

  // Adds items from this subsystem to the sendable builder, so that they get
  // sent to the driver station via network tables.
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Temp: HDrive", () -> m_driveTemp, null);
    builder.addBooleanProperty("Temp:: HDriveGood",
        () -> m_driveTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Position", this::getPosition, null);
  }

  // This method is called every 20 ms.
  @Override
  public void periodic() {
    // Update the motor temperature when the count matches the motor ID.
    if(Robot.getPeriodicCount() == Constants.HDrive.kMotor) {
      m_driveTemp = m_drive.getMotorTemperature();
    }
  }

  /**
   * Runs the H-drive.
   *
   * @param speed the desired sideways speed of the robot. Positive speed is to
   *              the right.
   */
  public void run(double speed) {
    m_drive.set(-speed);
  }

  /**
   * Stops the H-drive.
   */
  public void stop() {
    m_drive.stopMotor();
  }

  /**
   * Resets the position of the encoder to zero.
   */
  public void resetPosition() {
    m_encoder.setPosition(0);
  }

  /**
   * Gets the current position of the H-drive.
   *
   * @return the position of the H-drive, in meters.
   */
  public double getPosition() {
    return -m_encoder.getPosition();
  }

  /**
   * Gets the amount of current flowing through the motor.
   *
   * @return the current in Amperes.
   */
  public double getCurrent() {
    return(m_drive.getOutputCurrent());
  }
}