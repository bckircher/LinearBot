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
 * This subsystem controls the linear elevator.
 *
 * <p>When run normally, the left side of the elevator is driven at the
 * requested speed and the right side is run at a potentially different speed
 * with the goal of keeping the position of the two sides equal.
 */
public class Elevator extends SubsystemBase {
  /**
   * The motor controller for the left side of the linear elevator.
   */
  private final CANSparkMax m_left;

  /**
   * The motor controller for the right side of the linear elevator.
   */
  private final CANSparkMax m_right;

  /**
   * The encoder that tracks the position of the left side of the linear
   * elevator.
   */
  private final CANEncoder m_leftEncoder;

  /**
   * The encoder that tracks the position of the right side of the linear
   * elevator.
   */
  private final CANEncoder m_rightEncoder;

  /**
   * The temperature of the motor on the left side of the linear elevator.
   */
  private double m_leftTemp = 0;

  /**
   * The temperature of the motor on the right side of the linear elevator.
   */
  private double m_rightTemp = 0;

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
        add("Left Applied Voltage", m_left.getAppliedOutput());
        add("Left Current", m_left.getOutputCurrent());
        add("Left Temperature", m_left.getMotorTemperature());
        add("Left SPARK MAX Stick Faults", m_left.getStickyFaults());
        add("Left Position", m_leftEncoder.getPosition());
        add("Right Applied Voltage", m_right.getAppliedOutput());
        add("Right Current", m_right.getOutputCurrent());
        add("Right Temperature", m_right.getMotorTemperature());
        add("Right SPARK MAX Stick Faults", m_right.getStickyFaults());
        add("Right Position", m_rightEncoder.getPosition());
      }
    };
  */

  /**
   * Creates a new Elevator.
   *
   * @param left the motor controller for the left side of the elevator.
   *
   * @param right the motor controller for the right side of the elevator.
   *
   * @param leftEncoder the encoder that tracks the position of the left side
   *                    of the elevator.
   *
   * @param rightEncoder the encoder that tracks the position of the right side
   *                     of the elevator.
   */
  public Elevator(CANSparkMax left, CANSparkMax right, CANEncoder leftEncoder,
                  CANEncoder rightEncoder) {
    // Save the subsystem objects.
    m_left = left;
    m_right = right;
    m_leftEncoder = leftEncoder;
    m_rightEncoder = rightEncoder;

    // Configure the left motor controller.
    m_left.restoreFactoryDefaults();
    m_left.setOpenLoopRampRate(1);
    m_left.setIdleMode(IdleMode.kBrake);

    // Configure the right motor controller.
    m_right.restoreFactoryDefaults();
    m_right.setOpenLoopRampRate(1);
    m_right.setIdleMode(IdleMode.kBrake);

    // Set the conversion factor for the encoders so that they return the
    // position in meters.
    m_leftEncoder.setPositionConversionFactor(Constants.Elevator.
                                                kEncoderConversion);
    m_rightEncoder.setPositionConversionFactor(Constants.Elevator.
                                                 kEncoderConversion);

    // Reset the position of the encoders.
    resetPosition();
  }

  /**
   * Creates a new Elevator object.
   *
   * @return The new Elevator object.
   */
  public static Elevator create() {
    // Create the SparkMax controllers and encoders for use by the Elevator.
    CANSparkMax left =
      new CANSparkMax(Constants.Elevator.kMotorLeft, MotorType.kBrushless);
    CANSparkMax right =
      new CANSparkMax(Constants.Elevator.kMotorRight, MotorType.kBrushless);
    CANEncoder leftEncoder = left.getEncoder();
    CANEncoder rightEncoder = right.getEncoder();

    // Create the Elevator object.
    Elevator elevator = new Elevator(left, right, leftEncoder, rightEncoder);

    // Add periodic logging for the new Elevator object.
    new Logging.LoggingContext(elevator.getClass()) {
      @Override
      protected void addAll() {
        add("Left Applied Voltage", elevator.m_left.getAppliedOutput());
        add("Left Current", elevator.m_left.getOutputCurrent());
        add("Left Temperature", elevator.m_left.getMotorTemperature());
        add("Left SPARK MAX Stick Faults", elevator.m_left.getStickyFaults());
        add("Left Position", elevator.m_leftEncoder.getPosition());
        add("Right Applied Voltage", elevator.m_right.getAppliedOutput());
        add("Right Current", elevator.m_right.getOutputCurrent());
        add("Right Temperature", elevator.m_right.getMotorTemperature());
        add("Right SPARK MAX Stick Faults",
            elevator.m_right.getStickyFaults());
        add("Right Position", elevator.m_rightEncoder.getPosition());
      }
    };

    // Return the new Elevator object.
    return elevator;
  }

  // Adds items from this subsystem to the sendable builder, so that they get
  // sent to the driver station via network tables.
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Temp: Left", () -> m_leftTemp, null);
    builder.addBooleanProperty("Temp: LeftGood",
        () -> m_leftTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Left Position",
                              () -> m_leftEncoder.getPosition(), null);
    builder.addDoubleProperty("Temp: Right", () -> m_rightTemp, null);
    builder.addBooleanProperty("Temp:: RightGood",
        () -> m_rightTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Right Position",
                              () -> m_rightEncoder.getPosition(), null);
  }

  // This method is called every 20 ms.
  @Override
  public void periodic() {
    // Update the motor temperature when the count matches the motor ID.
    if(Robot.getPeriodicCount() == Constants.Elevator.kMotorLeft) {
      m_leftTemp = m_left.getMotorTemperature();
    }
    if(Robot.getPeriodicCount() == Constants.Elevator.kMotorRight) {
      m_rightTemp = m_right.getMotorTemperature();
    }
  }

  /**
   * Runs the elevator at the given speed.
   *
   * @param speed the speed at which the elevator should be run. Positive speed
   *              is up.
   */
  public void run(double speed) {
    double left, right;

    // Limit the speed to the maximum allowable speed.
    if(Math.abs(speed) > Constants.Elevator.kMaxSpeed) {
      speed = Math.copySign(Constants.Elevator.kMaxSpeed, speed);
    }

    // Determine how far apart the left and right sides of the elevator are
    // currently positioned.
    double delta = m_leftEncoder.getPosition() - m_rightEncoder.getPosition();

    // See if the elevator is moving down.
    if(speed < 0) {
      // See if the left side is lower than the right side.
      if(delta < 0) {
        // Moving down with the left side lower than the right side means that
        // the left side is leading the right side. Therefore, reduce the speed
        // of the left side and run the right side at the requested speed.
        left = speed - (delta * Constants.Elevator.kP);
        if(left > 0) {
          left = 0;
        }
        right = speed;
      }

      // Otherwise, the left side is higher than the right side.
      else {
        // Moving down with the left side higher than the right side means that
        // the left side is lagging the right side. Therefore, reduce the speed
        // of the right side and run the left side at the requested speed.
        left = speed;
        right = speed + (delta * Constants.Elevator.kP);
        if(right > 0) {
          right = 0;
        }
      }
    }

    // Otherwise, the elevator is moving up.
    else {
      // See if the left side is lower than the right side.
      if(delta < 0) {
        // Moving up with the left side lower than the right side means that
        // the left side is lagging the right side. Therefore, reduce the speed
        // of the right side and run the left side at the requested speed.
        left = speed;
        right = speed + (delta * Constants.Elevator.kP);
        if(right < 0) {
          right = 0;
        }
      }

      // Otherwise, the left side is higher than the right side.
      else {
        // Moving up with the left side higher than the right side means that
        // the left side is leading the right side. Therefore, reduce the speed
        // of the left side and run the right side at the requested speed.
        left = speed - (delta * Constants.Elevator.kP);
        if(left < 0) {
          left = 0;
        }
        right = speed;
      }
    }

    // Set the speed of the elevator.
    m_left.set(left);
    m_right.set(right);
  }

  /**
   * Directly runs the elevator motors at the requested speeds.
   *
   * <p>This method bypasses the auto-leveling controls, allowing the sides to
   * be individually controlled. The most helpful use of this is to manually
   * level the elevator.
   *
   * @param left the speed to run the left side of the elevator. Positive speed
   *             is up.
   *
   * @param right the speed to run the right side of the elevator. Positive
   *              speed is up.
   */
  public void run(double left, double right) {
    m_left.set(left);
    m_right.set(right);
  }

  /**
   * Directly sets the voltage of the elevator motors.
   *
   * @param left the voltage to apply to the left side of the elevator.
   *             Positive voltage is up.
   *
   * @param right the voltage to apply to the right side of the elevator.
   *              Positive voltage is up.
   */
  public void runVoltage(double left, double right) {
    m_left.setVoltage(left);
    m_right.setVoltage(right);
  }

  /**
   * Stops the elevator.
   */
  public void stop() {
    m_left.stopMotor();
    m_right.stopMotor();
  }

  /**
   * Resets the position of the encoders to zero.
   *
   * <p>The most helpful use of this is to reset the encoders when the elevator
   * is level and at the bottom of its travel (as determined by outside means).
   */
  public void resetPosition() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the current position of the elevator.
   *
   * <p>The position is determined by averaging the position of the left and
   * right side of the elevator.
   *
   * @return the position of the elevator, in meters.
   */
  public double getPosition() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
  }
}
