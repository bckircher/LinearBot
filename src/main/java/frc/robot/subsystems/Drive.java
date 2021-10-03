// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.NavX;
import frc.robot.utils.frc4048.Logging;

/**
 * This subsystem controls the drive train.
 */
public class Drive extends SubsystemBase {
  /**
   * The {@link NavX} to use to determine the orientation of the robot.
   */
  private final NavX m_navX;

  /**
   * The motor that drives the front left wheel.
   */
  private final CANSparkMax m_leftFront;

  /**
   * The motor that drives the back left wheel.
   */
  private final CANSparkMax m_leftBack;

  /**
   * The motor that drives the front right wheel.
   */
  private final CANSparkMax m_rightFront;

  /**
   * The motor that drives the back right wheel.
   */
  private final CANSparkMax m_rightBack;

  /**
   * The encoder that tracks the position of the left side of the drive train.
   * The rear motor/wheel is used since there is more weight on the back of the
   * robot, making this wheel less likely to slip.
   */
  private final CANEncoder m_leftEncoder;

  /**
   * The encoder that tracks the position of the right side of the drive train.
   * The rear motor/wheel is used since there is more weight on the back of the
   * robot. making this wheel less likely to slip.
   */
  private final CANEncoder m_rightEncoder;

  /**
   * The {@link DifferentialDrive} object that manages the drive train.
   */
  private final DifferentialDrive m_drive;

  /**
   * The {@link DifferentialDriveOdometry} object that tracks the position and
   * pose of the robot.
   */
  private final DifferentialDriveOdometry m_odometry;

  /**
   * The {@link Field2d} object that represents the robot's current position.
   */
  private final Field2d m_field = new Field2d();

  /**
   * The temperature of the front left motor of the drive train.
   */
  private double m_leftFrontTemp = 0;

  /**
   * The temperature of the back left motor of the drive train.
   */
  private double m_leftBackTemp = 0;

  /**
   * The temperature of the front right motor of the drive train.
   */
  private double m_rightFrontTemp = 0;

  /**
   * The temperature of the back right motor of the drive train.
   */
  private double m_rightBackTemp = 0;

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
        add("Left Front Applied Voltage", m_leftFront.getAppliedOutput());
        add("Left Front Current", m_leftFront.getOutputCurrent());
        add("Left Front Temperature", m_leftFront.getMotorTemperature());
        add("Left Front SPARK MAX Sticky Faults",
            m_leftFront.getStickyFaults());
        add("Left Back Applied Voltage", m_leftBack.getAppliedOutput());
        add("Left Back Current", m_leftBack.getOutputCurrent());
        add("Left Back Temperature", m_leftBack.getMotorTemperature());
        add("Left Back SPARK MAX Sticky Faults", m_leftBack.getStickyFaults());
        add("Left Position", m_leftEncoder.getPosition());
        add("Right Front Applied Voltage", m_rightFront.getAppliedOutput());
        add("Right Front Current", m_rightFront.getOutputCurrent());
        add("Right Front Temperature", m_rightFront.getMotorTemperature());
        add("Right Front SPARK MAX Sticky Faults",
            m_rightFront.getStickyFaults());
        add("Right Back Applied Voltage", m_rightBack.getAppliedOutput());
        add("Right Back Current", m_rightBack.getOutputCurrent());
        add("Right Back Temperature", m_rightBack.getMotorTemperature());
        add("Right Back SPARK MAX Sticky Faults",
            m_rightBack.getStickyFaults());
        add("Right Position", -m_rightEncoder.getPosition());
      }
    };
  */

  /**
   * Creates a new Drive.
   *
   * @param navX the {@link NavX} object to use to determine the robot's
   *             orientation.
   */
  public Drive(NavX navX, CANSparkMax leftFront, CANSparkMax leftBack,
               CANSparkMax rightFront, CANSparkMax rightBack,
               CANEncoder leftEncoder, CANEncoder rightEncoder) {
    // Save the subsystem objects.
    m_navX = navX;
    m_leftFront = leftFront;
    m_leftBack = leftBack;
    m_rightFront = rightFront;
    m_rightBack = rightBack;
    m_leftEncoder = leftEncoder;
    m_rightEncoder = rightEncoder;

    // Configure the front left motor controller.
    m_leftFront.restoreFactoryDefaults();
    m_leftFront.setIdleMode(IdleMode.kBrake);

    // Configure the back left motor controller.
    m_leftBack.restoreFactoryDefaults();
    m_leftBack.setIdleMode(IdleMode.kBrake);

    // Set the back left motor controller to do whatever the front left motor
    // controller is told to do (in other words, follow).
    m_leftBack.follow(m_leftFront);

    // Configure the front right motor controller.
    m_rightFront.restoreFactoryDefaults();
    m_rightFront.setIdleMode(IdleMode.kBrake);

    // Configure the back right motor controller.
    m_rightBack.restoreFactoryDefaults();
    m_rightBack.setIdleMode(IdleMode.kBrake);

    // Set the back right motor controller to do whatever the front right motor
    // controller is told to do (in other words, follow).
    m_rightBack.follow(m_rightFront);

    // Create the differential drive object that controls the drive train.
    m_drive = new DifferentialDrive(m_leftFront, m_rightFront);

    // Set the conversion factor for the encoders so that they return the
    // position in meters.
    m_leftEncoder.setPositionConversionFactor(Constants.Drive.
                                                kEncoderConversion);
    m_rightEncoder.setPositionConversionFactor(Constants.Drive.
                                                 kEncoderConversion);

    // Set the conversion factor for the encoders so that they return the
    // velocity in meters per second.
    m_leftEncoder.setVelocityConversionFactor(Constants.Drive.
                                                kEncoderConversion / 60);
    m_rightEncoder.setVelocityConversionFactor(Constants.Drive.
                                                 kEncoderConversion / 60);

    // Reset the position of the encoders.
    resetPosition();

    // Create the object to track the odometry of the robot, based on the
    // current orientation.
    m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2D());

    // Configure the motor safety of the differential drive object.
    m_drive.setExpiration(0.1);
    m_drive.setSafetyEnabled(true);
    m_drive.setDeadband(0.0);

    // Add the differential drive object to SmartDashboard, allowing it to be
    // viewed on the driver station.
    addChild("Differential Drive", m_drive);

    // Add the field position object to SmartDashboard, allowing it to be
    // viewed on the driver station.
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Creates a new Drive.
   *
   * <p>This is a wrapper around `new` and the constructor; it creates the
   * hardware entities used by the Drive subsystem and passes them to the
   * constructor.
   *
   * @param navX the {@link NavX} object to use to determine the robot's
   *             orientation.
   *
   * @return The newly allocated Drive object.
   */
  public static Drive create(NavX navX) {
    // Create the motor controllers used by the Drive subsystem.
    CANSparkMax leftFront =
      new CANSparkMax(Constants.Drive.kMotorLeftFront, MotorType.kBrushless);
    CANSparkMax leftBack =
      new CANSparkMax(Constants.Drive.kMotorLeftBack, MotorType.kBrushless);
    CANSparkMax rightFront =
      new CANSparkMax(Constants.Drive.kMotorRightFront, MotorType.kBrushless);
    CANSparkMax rightBack =
      new CANSparkMax(Constants.Drive.kMotorRightBack, MotorType.kBrushless);

    // Create the encoders used by the Drive subsystem.
    CANEncoder leftEncoder = leftBack.getEncoder();
    CANEncoder rightEncoder = rightBack.getEncoder();

    // Create the Drive object.
    Drive drive = new Drive(navX, leftFront, leftBack, rightFront, rightBack,
                            leftEncoder, rightEncoder);

    // Add periodic logging for the new Drive object.
    new Logging.LoggingContext(drive.getClass()) {
      @Override
      protected void addAll() {
        add("Left Front Applied Voltage",
            drive.m_leftFront.getAppliedOutput());
        add("Left Front Current", drive.m_leftFront.getOutputCurrent());
        add("Left Front Temperature", drive.m_leftFront.getMotorTemperature());
        add("Left Front SPARK MAX Sticky Faults",
            drive.m_leftFront.getStickyFaults());
        add("Left Back Applied Voltage", drive.m_leftBack.getAppliedOutput());
        add("Left Back Current", drive.m_leftBack.getOutputCurrent());
        add("Left Back Temperature", drive.m_leftBack.getMotorTemperature());
        add("Left Back SPARK MAX Sticky Faults",
            drive.m_leftBack.getStickyFaults());
        add("Left Position", drive.m_leftEncoder.getPosition());
        add("Right Front Applied Voltage",
            drive.m_rightFront.getAppliedOutput());
        add("Right Front Current", drive.m_rightFront.getOutputCurrent());
        add("Right Front Temperature",
            drive.m_rightFront.getMotorTemperature());
        add("Right Front SPARK MAX Sticky Faults",
            drive.m_rightFront.getStickyFaults());
        add("Right Back Applied Voltage",
            drive.m_rightBack.getAppliedOutput());
        add("Right Back Current", drive.m_rightBack.getOutputCurrent());
        add("Right Back Temperature", drive.m_rightBack.getMotorTemperature());
        add("Right Back SPARK MAX Sticky Faults",
            drive.m_rightBack.getStickyFaults());
        add("Right Position", -drive.m_rightEncoder.getPosition());
      }
    };

    // Return the new Drive object.
    return drive;
  }

  // Adds items from this subsystem to the sendable builder, so that they get
  // sent to the driver station via network tables.
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Temp: LeftFront", () -> m_leftFrontTemp, null);
    builder.addBooleanProperty("Temp: LeftFrontGood",
        () -> m_leftFrontTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Temp: LeftBack", () -> m_leftBackTemp, null);
    builder.addBooleanProperty("Temp: LeftBackGood",
        () -> m_leftBackTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Left Position",
                              () -> m_leftEncoder.getPosition(), null);
    builder.addDoubleProperty("Left Velocity",
                              () -> m_leftEncoder.getVelocity(), null);
    builder.addDoubleProperty("Temp: RightFront",
        () -> m_rightFrontTemp, null);
    builder.addBooleanProperty("Temp: RightFrontGood",
        () -> m_rightFrontTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Temp: RightBack", () -> m_rightBackTemp, null);
    builder.addBooleanProperty("Temp: RightBackGood",
        () -> m_rightBackTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Right Position",
                              () -> -m_rightEncoder.getPosition(), null);
    builder.addDoubleProperty("Right Velocity",
                              () -> -m_rightEncoder.getVelocity(), null);
  }

  // This method is called every 20 ms.
  @Override
  public void periodic() {
    // Update the odometry of the robot based on the current orientation and
    // position of the motors.
    m_odometry.update(m_navX.getRotation2D(), m_leftEncoder.getPosition(),
                      -m_rightEncoder.getPosition());

    // Update the current robot position on the field.
    m_field.setRobotPose(m_odometry.getPoseMeters());





    Pose2d pose = m_odometry.getPoseMeters();
    SmartDashboard.putNumber("pose_x", pose.getX());
    SmartDashboard.putNumber("pose_y", pose.getY());
    SmartDashboard.putNumber("pose_r", pose.getRotation().getDegrees());





    // Update the motor temperature when the count matches the motor ID.
    if(Robot.getPeriodicCount() == Constants.Drive.kMotorLeftFront) {
      m_leftFrontTemp = m_leftFront.getMotorTemperature();
    }
    if(Robot.getPeriodicCount() == Constants.Drive.kMotorLeftBack) {
      m_leftBackTemp = m_leftBack.getMotorTemperature();
    }
    if(Robot.getPeriodicCount() == Constants.Drive.kMotorRightFront) {
      m_rightFrontTemp = m_rightFront.getMotorTemperature();
    }
    if(Robot.getPeriodicCount() == Constants.Drive.kMotorRightBack) {
      m_rightBackTemp = m_rightBack.getMotorTemperature();
    }
  }

  /**
   * Drives the robot in arcade mode.
   *
   * @param speed the desired forward speed of the robot. Positive speed is
   *              forward.
   *
   * @param rotation the desired rotation of the robot. Position rotation is
   *                 clockwise.
   */
  public void driveArcade(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation, true);
  }

  /**
   * Drives the robot in tank mode.
   *
   * @param left the desired speed for the left motors. Positive speed is
   *             forward.
   *
   * @param right the desired speed for the right motors. Positive speed is
   *              forward.
   */
  public void driveTank(double left, double right) {
    m_drive.tankDrive(left, right, true);
  }

  /**
   * Drives the robot in tank mode, using voltage.
   *
   * @param leftVolts the desired voltage for the left motors. Positive voltage
   *                  is forward.
   *
   * @param rightVolts the desired voltage for the right motors. Positive
   *                   voltage is forward.
   */
  public void driveTankVolts(double leftVolts, double rightVolts) {
    m_leftFront.setVoltage(leftVolts);
    m_rightFront.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Stops the drive train.
   */
  public void stop() {
    m_drive.stopMotor();
  }

  /**
   * Resets the position of the encoders to zero.
   */
  public void resetPosition() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the current position of the drive train.
   *
   * <p>The position is determined by averaging the position of the left and
   * right side of the drive train.
   *
   * @return the position of the drive train, in meters.
   */
  public double getPosition() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
  }

  /**
   * Gets the speed of the wheels.
   *
   * @return the {@link DifferentialDriveWheelSpeeds} that provides the current
   *         wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),
                                            -m_rightEncoder.getVelocity());
  }

  /**
   * Gets the current position of the robot on the field.
   *
   * @return the @{link Pose2d} that provides the current position of the robot
   *         on the field.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry of the robot.
   *
   * @param pose the position of the robot on the field.
   */
  public void resetOdometry(Pose2d pose) {
    // Reset the position encoders.
    resetPosition();

    // Reset the odometry to the given position and the current heading from
    // the NavX.
    m_odometry.resetPosition(pose, m_navX.getRotation2D());
  }
}