// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.util.stream.Stream;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.Mockito;

// JUnit test of the HDrive subsystem class.
public class testHDrive {
  // Create a mock of the motor controller.
  private final CANSparkMax m_motor = mock(CANSparkMax.class);

  // Create a mock of the encoder.
  private final CANEncoder m_encoder = mock(CANEncoder.class);

  // Create an instance of the HDrive subsystem.
  private final HDrive m_hDrive = new HDrive(m_motor, m_encoder);

  // A unit test for simple driving of the HDrive.
  @ParameterizedTest
  @MethodSource
  void run(double speed) {
    // Reset the motor controller mock object.
    Mockito.reset(m_motor);

    // Run the HDrive at the requested speed.
    m_hDrive.run(speed);

    // Verify that the motor controller was set to go the given speed. Because
    // of the way the HDrive is mounted, making positive mean right requires
    // inverting the motor drive.
    verify(m_motor).set(-speed);
  }

  // Generates a test data set for the simple drive test.
  static Stream<Arguments> run() {
    // Create a stream builder for storing all the test data sets.
    Stream.Builder<Arguments> builder = Stream.builder();

    // Loop through the values of the simple drive test.
    for(int i = -10; i <= 10; i++) {
      // Add a data set for this value.
      builder.add(Arguments.of(i / 10.0));
    }

    // Add a variety of randomized speeds.
    for(int i = 0; i < 100; i++) {
      // Add a data set for a random speed.
      builder.add(Arguments.of((Math.random() * 2) - 1));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  @ParameterizedTest
  @MethodSource
  void runVoltage(double volts) {
    // Reset the motor controller mock object.
    Mockito.reset(m_motor);

    // Run the HDrive at the requested voltage.
    m_hDrive.runVoltage(volts);

    // Verify that the motor controller was set to output the given voltage.
    // Because of the way the HDrive is mounted, making positive mean right
    // requires inverting the motor drive.
    verify(m_motor).setVoltage(-volts);
  }

  static Stream<Arguments> runVoltage() {
    // Create a stream builder for storing all the test data sets.
    Stream.Builder<Arguments> builder = Stream.builder();

    // Loop through the values of the simple drive test.
    for(int i = -10; i <= 10; i++) {
      // Add a data set for this value.
      builder.add(Arguments.of(i / 10.0));
    }

    // Add a variety of randomized speeds.
    for(int i = 0; i < 100; i++) {
      // Add a data set for a random speed.
      builder.add(Arguments.of((Math.random() * 2) - 1));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  // A unit test for stopping the HDrive.
  @Test
  void stop() {
    // Reset the motor controller mock object.
    Mockito.reset(m_motor);

    // Stop the hDrive.
    m_hDrive.stop();

    // Verify that the motor controller was stopped.
    verify(m_motor).stopMotor();
  }

  // A unit test for checking the position from the encoder.
  @ParameterizedTest
  @MethodSource
  void encoder(double position) {
    // Reset the encoder mock object.
    Mockito.reset(m_encoder);

    // Set the value that the encoder will return.
    when(m_encoder.getPosition()).thenReturn(position);

    // Get the position from the HDrive object and make sure it is close enough
    // to the expected value. Because of the way the HDrive is mounted, making
    // positive mean right requires inverting the encoder, hence the returned
    // and expected positions are added (since one should be the inverse of the
    // other).
    assertTrue(Math.abs(m_hDrive.getPosition() + position) < 0.0001);

    // Verify that the encoder position was actually queried.
    verify(m_encoder).getPosition();
  }

  // Generates a test data set for the encoder position test.
  static Stream<Arguments> encoder() {
     // Create a stream builder for storing all the test data sets.
     Stream.Builder<Arguments> builder = Stream.builder();

     // Loop through the values of the encoder position test.
     for(int i = -986; i <= -986; i += 17) {
      // Add a data set for this value.
      builder.add(Arguments.of(i / 100.0));
    }

    // Add a variety of randomized speeds.
    for(int i = 0; i < 100; i++) {
      // Add a data set for a random speed.
      builder.add(Arguments.of((Math.random() * 20) - 10));
    }

     // Create and return an argument stream of these data sets.
     return builder.build();
  }

  // A unit test for resetting the encoder.
  @Test
  void reset() {
    // Reset the encoder mock object.
    Mockito.reset(m_encoder);

    // Reset the HDrive position.
    m_hDrive.resetPosition();

    // Verify that the encoder position was reset.
    verify(m_encoder).setPosition(0);
  }
}
