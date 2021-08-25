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
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.NavX;
import frc.robot.utils.frc4048.Logging;

public class Drive extends SubsystemBase {
  private final NavX m_navX;

  private final CANSparkMax m_leftFront =
    new CANSparkMax(Constants.Drive.kMotorLeftFront, MotorType.kBrushless);
  private final CANSparkMax m_leftBack =
    new CANSparkMax(Constants.Drive.kMotorLeftBack, MotorType.kBrushless);
  private final CANSparkMax m_rightFront =
    new CANSparkMax(Constants.Drive.kMotorRightFront, MotorType.kBrushless);
  private final CANSparkMax m_rightBack =
    new CANSparkMax(Constants.Drive.kMotorRightBack, MotorType.kBrushless);

  private final CANEncoder m_leftEncoder = m_leftBack.getEncoder();
  private final CANEncoder m_rightEncoder = m_rightBack.getEncoder();

  private final DifferentialDrive m_drive =
    new DifferentialDrive(m_leftFront, m_rightFront);

  private final DifferentialDriveOdometry m_odometry;

  private double m_leftFrontTemp = 0;
  private double m_leftBackTemp = 0;
  private double m_rightFrontTemp = 0;
  private double m_rightBackTemp = 0;

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
      }
    };

  /** Creates a new Drive. */
  public Drive(NavX navX) {
    m_navX = navX;

    m_leftFront.restoreFactoryDefaults();
    m_leftFront.setIdleMode(IdleMode.kBrake);

    m_leftBack.restoreFactoryDefaults();
    m_leftBack.setIdleMode(IdleMode.kBrake);

    m_leftBack.follow(m_leftFront);

    m_rightFront.restoreFactoryDefaults();
    m_rightFront.setIdleMode(IdleMode.kBrake);

    m_rightBack.restoreFactoryDefaults();
    m_rightBack.setIdleMode(IdleMode.kBrake);
  
    m_rightBack.follow(m_rightFront);

    m_leftEncoder.setPositionConversionFactor(Constants.Drive.
                                                kEncoderConversion);
    m_rightEncoder.setPositionConversionFactor(Constants.Drive.
                                                 kEncoderConversion);

    resetPosition();
    m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2D());

    m_drive.setSafetyEnabled(true);
    m_drive.setExpiration(0.1);

    SmartDashboard.putData(this);

    addChild("Differential Drive", m_drive);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Temp: LeftFront", () -> m_leftFrontTemp, null);
    builder.addBooleanProperty("Temp: LeftFrontGood",
        () -> m_leftFrontTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Temp: LeftBack", () -> m_leftBackTemp, null);
    builder.addBooleanProperty("Temp: LeftBackGood",
        () -> m_leftBackTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Temp: RightFront",
        () -> m_rightFrontTemp, null);
    builder.addBooleanProperty("Temp: RightFrontGood",
        () -> m_rightFrontTemp < Constants.maxMotorTemperature, null);
    builder.addDoubleProperty("Temp: RightBack", () -> m_rightBackTemp, null);
    builder.addBooleanProperty("Temp: RightBackGood",
        () -> m_rightBackTemp < Constants.maxMotorTemperature, null);
  }

  @Override
  public void periodic() {
    m_odometry.update(m_navX.getRotation2D(), m_leftEncoder.getPosition(),
                      m_rightEncoder.getPosition());

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

  public void driveArcade(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation, true);
  }

  public void driveTank(double left, double right) {
    m_drive.tankDrive(left, right, true);
  }

  public void driveTankVolts(double leftVolts, double rightVolts) {
    m_leftFront.setVoltage(leftVolts);
    m_rightFront.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void stop() {
    m_drive.stopMotor();
  }

  public void resetPosition() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getPosition() {
    return((m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),
                                            m_rightEncoder.getVelocity());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetPosition();
    m_odometry.resetPosition(pose, m_navX.getRotation2D());
  }
}
