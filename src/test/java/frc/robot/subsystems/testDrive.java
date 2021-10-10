// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.util.stream.Stream;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.utils.NavX;

// JUnit test of the Drive subsystem class.
public class testDrive {
  // The maximum allowable error for numerical checks.
  private final static double m_kError = 0.00001;

  // Create a mock of the navX.
  private final NavX m_navX = mock(NavX.class);

  // Create a mock of the front left motor controller.
  private final CANSparkMax m_leftFront = mock(CANSparkMax.class);

  // Create a mock of the back left motor controller.
  private final CANSparkMax m_leftBack = mock(CANSparkMax.class);

  // Create a mock of the front right motor controller.
  private final CANSparkMax m_rightFront = mock(CANSparkMax.class);

  // Create a mock of the back right motor controller.
  private final CANSparkMax m_rightBack = mock(CANSparkMax.class);

  // Create a mock of the left encoder.
  private final CANEncoder m_leftEncoder = mock(CANEncoder.class);

  // Create a mock of the right encoder.
  private final CANEncoder m_rightEncoder = mock(CANEncoder.class);

  // A unit test for arcade drive mode.
  @ParameterizedTest
  @MethodSource
  void driveArcade(double speed, double rotation, double left, double right) {
    Drive drive;

    // Create a captor for saving the values that are passed to the motor
    // controllers.
    ArgumentCaptor<Double> leftCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rightCaptor = ArgumentCaptor.forClass(Double.class);

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the motor controllers.
    Mockito.reset(m_leftFront);
    Mockito.reset(m_leftBack);
    Mockito.reset(m_rightFront);
    Mockito.reset(m_rightBack);

    // Run the drive in arcade mode.
    drive.driveArcade(speed, rotation);

    // Verify that the appropriate motor controllers were set, as well as that
    // the followers were not set (since they are handled via following).
    verify(m_leftFront).set(leftCaptor.capture());
    verify(m_leftBack, never()).set(0);
    verify(m_rightFront).set(rightCaptor.capture());
    verify(m_rightBack, never()).set(0);

    // Verify that the speeds of the two motors were set correctly.
    assertTrue(Math.abs(left - leftCaptor.getValue()) < m_kError);
    assertTrue(Math.abs(right - rightCaptor.getValue()) < m_kError);
  }

  // Generates a test data set for the arcade drive mode test.
  static Stream<Arguments> driveArcade() {
    Stream.Builder<Arguments> builder = Stream.builder();
    double speed, steer, max, left, right;

    // Create test data sets for varying speeds.
    for(int i = -10; i <= 10; i++) {
      builder.add(Arguments.of(i / 10.0, 0.0, Math.copySign(i * i, i) / 100.0,
                               -Math.copySign(i * i, i) / 100.0));
    }

    // Create test data sets for varying steering.
    for(int i = -10; i <= 10; i++) {
      builder.add(Arguments.of(0.0, i / 10.0, Math.copySign(i * i, i) / 100.0,
                               Math.copySign(i * i, i) / 100.0));
    }

    // Create test data sets for varying steering while driving forward.
    for(int i = -10; i <= 10; i++) {
      if(i < 0) {
        left = 1.0 - ((i * i) / 100.0);
        right = -1.0;
     } else {
        left = 1.0;
        right = -1.0 + ((i * i) / 100.0);
      }
      builder.add(Arguments.of(1.0, i / 10.0, left, right));
    }

    // Create randomized test data sets.
    for(int i = 0; i < 100; i++) {
      speed = (Math.random() * 2) - 1;
      steer = (Math.random() * 2) - 1;

      max = Math.max(speed * speed, steer * steer);

      if(speed < 0) {
        if(steer < 0) {
          left = -max;
          right = (speed * speed) - (steer * steer);
        } else {
          left = (steer * steer) - (speed * speed);
          right = max;
        }
      } else {
        if(steer < 0) {
          left = (speed * speed) - (steer * steer);
          right = -max;
        } else {
          left = max;
          right = (steer * steer) - (speed * speed);
        }
      }

      builder.add(Arguments.of(speed, steer, left, right));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  // A unit test for tank drive mode.
  @ParameterizedTest
  @MethodSource
  void driveTank(double left, double right) {
    Drive drive;

    // Create a captor for saving the values that are passed to the motor
    // controllers.
    ArgumentCaptor<Double> leftCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rightCaptor = ArgumentCaptor.forClass(Double.class);

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the motor controllers.
    Mockito.reset(m_leftFront);
    Mockito.reset(m_leftBack);
    Mockito.reset(m_rightFront);
    Mockito.reset(m_rightBack);

    // Run the drive in tank mode.
    drive.driveTank(left, right);

    // Verify that the appropriate motor controllers were set, as well as that
    // the followers were not set (since they are handled via following).
    verify(m_leftFront).set(leftCaptor.capture());
    verify(m_leftBack, never()).set(0);
    verify(m_rightFront).set(rightCaptor.capture());
    verify(m_rightBack, never()).set(0);

    // Verify that the speeds of the two motors were set correctly.
    assertTrue(Math.abs(Math.copySign(left * left, left) -
                        leftCaptor.getValue()) < m_kError);
    assertTrue(Math.abs(Math.copySign(right * right, right) +
                        rightCaptor.getValue()) < m_kError);
  }

  // Generates a test data set for the tank drive mode test.
  static Stream<Arguments> driveTank() {
    Stream.Builder<Arguments> builder = Stream.builder();

    // Create test data sets for various speeds of the two motors.
    for(int i = -10; i <= 10; i++) {
      for(int j = -10; j <= 10; j++) {
        builder.add(Arguments.of(i / 10.0, j / 10.0));
      }
    }

    // Create randomized test data sets.
    for(int i = 0; i < 100; i++) {
      builder.add(Arguments.of((Math.random() * 2) - 1,
                               (Math.random() * 2) - 1));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  // A unit test for tank voltage mode.
  @ParameterizedTest
  @MethodSource
  void driveTankVolts(double left, double right) {
    Drive drive;

    // Create a captor for saving the values that are passed to the motor
    // controllers.
    ArgumentCaptor<Double> leftCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rightCaptor = ArgumentCaptor.forClass(Double.class);

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the motor controllers.
    Mockito.reset(m_leftFront);
    Mockito.reset(m_leftBack);
    Mockito.reset(m_rightFront);
    Mockito.reset(m_rightBack);

    // Run the drive in tank voltage mode.
    drive.driveTankVolts(left, right);

    // Verify that the appropriate motor controllers were set, as well as that
    // the followers were not set (since they are handled via following).
    verify(m_leftFront).setVoltage(leftCaptor.capture());
    verify(m_leftBack, never()).setVoltage(0);
    verify(m_rightFront).setVoltage(rightCaptor.capture());
    verify(m_rightBack, never()).setVoltage(0);

    // Verify that the speeds of the two motors were set correctly.
    assertTrue(Math.abs(left - leftCaptor.getValue()) < m_kError);
    assertTrue(Math.abs(right + rightCaptor.getValue()) < m_kError);
  }

  // Generates a test data set for the tank voltage mode test.
  static Stream<Arguments> driveTankVolts() {
    Stream.Builder<Arguments> builder = Stream.builder();

    // Create test data sets for various voltages of the two motors.
    for(int i = -10; i <= 10; i++) {
      for(int j = -10; j <= 10; j++) {
        builder.add(Arguments.of(i / 10.0, j / 10.0));
      }
    }

    // Create randomized test data sets.
    for(int i = 0; i < 100; i++) {
      builder.add(Arguments.of((Math.random() * 2) - 1,
                               (Math.random() * 2) - 1));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  // A unit test for stopping the drive.
  @Test
  void stop() {
    Drive drive;

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the motor controllers.
    Mockito.reset(m_leftFront);
    Mockito.reset(m_leftBack);
    Mockito.reset(m_rightFront);
    Mockito.reset(m_rightBack);

    // Stop the drive.
    drive.stop();

    // Verify that the appropriate motor controllers were set, as well as that
    // the followers were not set (since they are handled via following).
    verify(m_leftFront).stopMotor();
    verify(m_leftBack, never()).stopMotor();
    verify(m_rightFront).stopMotor();
    verify(m_rightBack, never()).stopMotor();
  }

  // A unit test for resetting the position of the drive.
  @Test
  void resetPosition() {
    Drive drive;

    // Create a captor for saving the values that are passed to the encoders.
    ArgumentCaptor<Double> leftCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rightCaptor = ArgumentCaptor.forClass(Double.class);

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the encoders.
    Mockito.reset(m_leftEncoder);
    Mockito.reset(m_rightEncoder);

    // Reset the position of the drive.
    drive.resetPosition();

    // Verify that the encoders were reset.
    verify(m_leftEncoder).setPosition(leftCaptor.capture());
    verify(m_rightEncoder).setPosition(rightCaptor.capture());
    assertTrue(leftCaptor.getValue() == 0.0);
    assertTrue(rightCaptor.getValue() == 0.0);
  }

  // A unit test for getting the position of the drive.
  @ParameterizedTest
  @MethodSource
  void getPosition(double left, double right, double value) {
    double result;
    Drive drive;

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the encoders.
    Mockito.reset(m_leftEncoder);
    Mockito.reset(m_rightEncoder);

    // Setup the encoders to return the given positions.
    when(m_leftEncoder.getPosition()).thenReturn(left);
    when(m_rightEncoder.getPosition()).thenReturn(right);

    // Get the position of the drive.
    result = drive.getPosition();

    // Verify that the encoders were both queried.
    verify(m_leftEncoder).getPosition();
    verify(m_rightEncoder).getPosition();

    // Verify that the expected position was returned.
    assertTrue(Math.abs(result - value) < m_kError);
  }

  // Generates a test data set for the get position test.
  static Stream<Arguments> getPosition() {
    Stream.Builder<Arguments> builder = Stream.builder();
    double left, right;

    // Create test data sets for various positions of the two encoders.
    for(int i = -10; i <= 10; i++) {
      for(int j = -10; j <= 10; j++) {
        builder.add(Arguments.of(i / 10.0, j / 10.0, (i - j) / 20.0));
      }
    }

     // Create randomized test data sets.
     for(int i = 0; i < 100; i++) {
      left = (Math.random() * 2) - 1;
      right = (Math.random() * 2) - 1;
      builder.add(Arguments.of(left, right, (left - right) / 2));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  @ParameterizedTest
  @MethodSource
  void getWheelSpeeds(double left, double right, double expectLeft,
                      double expectRight) {
    DifferentialDriveWheelSpeeds result;
    Drive drive;

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the encoders.
    Mockito.reset(m_leftEncoder);
    Mockito.reset(m_rightEncoder);

    // Setup the encoders to return the given velocities.
    when(m_leftEncoder.getVelocity()).thenReturn(left);
    when(m_rightEncoder.getVelocity()).thenReturn(right);

    // Get the position of the drive.
    result = drive.getWheelSpeeds();

    // Verify that the encoders were both queried.
    verify(m_leftEncoder).getVelocity();
    verify(m_rightEncoder).getVelocity();

    // Verify that the expected wheel speeds were returned.
    assertTrue(Math.abs(result.leftMetersPerSecond - expectLeft) < m_kError);
    assertTrue(Math.abs(result.rightMetersPerSecond - expectRight) < m_kError);
  }

  // Generates a test data set for the get wheel speed test.
  static Stream<Arguments> getWheelSpeeds() {
    Stream.Builder<Arguments> builder = Stream.builder();
    double left, right;

    // Create test data sets for various velocities of the two encoders.
    for(int i = -10; i <= 10; i++) {
      for(int j = -10; j <= 10; j++) {
        builder.add(Arguments.of(i / 10.0, j / 10.0, i / 10.0, -j / 10.0));
      }
    }

     // Create randomized test data sets.
     for(int i = 0; i < 100; i++) {
      left = (Math.random() * 2) - 1;
      right = (Math.random() * 2) - 1;

      builder.add(Arguments.of(left, right, left, -right));
    }

     // Create and return an argument stream of these data sets.
     return builder.build();
  }

  // A unit test for getting the pose of the robot.
  @ParameterizedTest
  @MethodSource
  void getPose(double encoderLeft, double encoderRight, double gyroAngle) {
    Drive drive;
    Pose2d pose;

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks of the navX and the two encoders.
    Mockito.reset(m_navX);
    Mockito.reset(m_leftEncoder);
    Mockito.reset(m_rightEncoder);

    // Set the mocks of the navX and the two encoders to return the given
    // values when queried.
    when(m_leftEncoder.getPosition()).thenReturn(encoderLeft);
    when(m_rightEncoder.getPosition()).thenReturn(-encoderRight);
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(gyroAngle));

    // Call the periodic function, which updates the pose.
    drive.periodic();

    // Get the updated robot pose.
    pose = drive.getPose();

    // Verify that the rotation matrix was queried from the navX and the
    // position was queried from the two encoders.
    verify(m_navX).getRotation2D();
    verify(m_leftEncoder).getPosition();
    verify(m_rightEncoder).getPosition();

    // Verify that the robot position is roughly as expected (either zero when
    // expected to be zero or non-zero when expected to be non-zero).
    if(encoderLeft != -encoderRight) {
      if(gyroAngle != 0.0) {
        assertTrue((pose.getX() != 0.0) || (pose.getY() != 0.0));
      } else {
        assertTrue((pose.getX() != 0.0) && (pose.getY() == 0.0));
      }
    } else {
      assertTrue((pose.getX() == 0.0) && (pose.getY() == 0.0));
    }

    // Clamp the gyro angle to the +/- PI range.
    while(gyroAngle > Math.PI) {
      gyroAngle -= 2.0 * Math.PI;
    }
    while(gyroAngle < -Math.PI) {
      gyroAngle += 2.0 * Math.PI;
    }

    // Verify that the gyro angle is correctly captured in the robot pose.
    assertTrue(Math.abs(pose.getRotation().getRadians() - gyroAngle) <
               m_kError);
  }

  // Generates a test data set for the get robot pose test.
  static Stream<Arguments> getPose() {
    Stream.Builder<Arguments> builder = Stream.builder();

    // Create test data sets for various positions of the two encoders and
    // robot rotation.
    for(int i = -51; i <= 51; i += 17) {
      for(int j = -51; j <= 51; j += 17) {
        for(int k = -51; k <= 51; k += 17) {
          builder.add(Arguments.of(i / 10.0, j / 10.0, k / 10.0));
        }

        // Add special cases for the angle being +/- PI.
        builder.add(Arguments.of(i / 10.0, j / 10.0, Math.PI));
        builder.add(Arguments.of(i / 10.0, j / 10.0, -Math.PI));
      }
    }

     // Create randomized test data sets.
     for(int i = 0; i < 100; i++) {
      builder.add(Arguments.of((Math.random() * 20.0) - 10.0,
                               (Math.random() * 20.0) - 10.0,
                               (Math.random() * 20.0) - 10.0));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  // A unit test for resetting the drive odometry.
  @ParameterizedTest
  @MethodSource
  void resetOdometry(double x, double y, double angle) {
    Drive drive;
    Pose2d pose;

    // Set the mocked navX to return a zero degree rotation matrix when
    // queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(0));

    // Create a drive subsystem to test.
    drive = new Drive(m_navX, m_leftFront, m_leftBack, m_rightFront,
                      m_rightBack, m_leftEncoder, m_rightEncoder);

    // Reset the mocks for the navX and the left/right encoders.
    Mockito.reset(m_navX);
    Mockito.reset(m_leftEncoder);
    Mockito.reset(m_rightEncoder);

    // Set the mocked navX to return the given rotation when queried.
    when(m_navX.getRotation2D()).thenReturn(new Rotation2d(angle));

    // Reset the drive odometry.
    drive.resetOdometry(new Pose2d(x, y, new Rotation2d(angle)));

    // Verify that the rotation matrix was queried from the navX and that the
    // left/right encoders were reset.
    verify(m_navX).getRotation2D();
    verify(m_leftEncoder).setPosition(0.0);
    verify(m_rightEncoder).setPosition(0.0);

    // Get the current robot pose.
    pose = drive.getPose();

    // Verify that the robot pose is correct.
    assertTrue(Math.abs(pose.getX() - x) < m_kError);
    assertTrue(Math.abs(pose.getY() - y) < m_kError);
    assertTrue(Math.abs(pose.getRotation().getRadians() - angle) < m_kError);
 }

  // Generates a test data set for the reset odometry test.
  static Stream<Arguments> resetOdometry() {
    Stream.Builder<Arguments> builder = Stream.builder();

    // Create test data sets for each of the components in the pose being
    // independent.
    for(int i = -100; i <= 100; i++) {
      builder.add(Arguments.of(i / 100.0, 0.0, 0.0));
      builder.add(Arguments.of(0.0, i / 100.0, 0.0));
      builder.add(Arguments.of(0.0, 0.0, i / 100.0));
    }

    // Create randomized test data sets.
    for(int i = 0; i < 100; i++) {
      builder.add(Arguments.of((Math.random() * 20) - 10,
                                (Math.random() * 20) - 10,
                                (Math.random() * 20) - 10));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }
}
