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

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

// JUnit test of the Intake subsystem class.
public class testIntake {
  // Create a mock of the power distribution panel.
  private final PowerDistributionPanel m_pdp =
    mock(PowerDistributionPanel.class);

  // Create a mock of the motor controller.
  private final Spark m_spark = mock(Spark.class);

  // Create an instance of the Intake subsystem.
  private final Intake m_intake = new Intake(m_pdp, m_spark);
  
  // A unit test for running the Intake in no current conditions.
  @ParameterizedTest
  @MethodSource
  void run(double speed) {
    // Reset the motor controller mock object.
    Mockito.reset(m_spark);

    // Set the PDP to return zero current for the intake motor's channel.
    when(m_pdp.getCurrent(Constants.PDP.kIntake)).thenReturn(0.0);

    // Set the speed of the intake and run a tick of the periodic method.
    m_intake.run(speed);
    m_intake.periodic();

    // See if the target speed is non-zero.
    if(speed != 0.0) {
      // Verify that the motor controller speed was set to the target speed.
      verify(m_spark).set(speed);
    } else {
      // Verify that the speed of the motor controller was not set.
      verify(m_spark, never()).set(0.0);
    }
  }

  // Generates a test data set for the no current test.
  static Stream<Arguments> run() {
    // Create a stream builder for storing all the test data sets.
    Stream.Builder<Arguments> builder = Stream.builder();

    // Loop through the values of the low/no current test.
    for(int i = -100; i <= 100; i++) {
      // Add a data set for this value.
      builder.add(Arguments.of(i / 100.0));
    }

    // Add a variety of randomized speeds.
    for(int i = 0; i < 100; i++) {
      // Add a data set for a random speed.
      builder.add(Arguments.of((Math.random() * 2) - 1));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }

  // A unit test for running the Intake in current limiting conditions.
  @ParameterizedTest
  @MethodSource
  void runLimited(double speed, double current) {
    // Create a captor for saving the value that is passed to the motor
    // controller.
    ArgumentCaptor<Double> speedCaptor = ArgumentCaptor.forClass(Double.class);

    // Reset the motor controller mock object.
    Mockito.reset(m_spark);

    // Set the PDP to return the given current for the intake motor's channel.
    when(m_pdp.getCurrent(Constants.PDP.kIntake)).thenReturn(current);

    // Set the speed of the intake and run a tick of the periodic method.
    m_intake.run(speed);
    m_intake.periodic();

    // See if the target speed is non-zero.
    if(speed != 0) {
      // Verify that the motor controller speed was set, and capture the value
      // that was used.
      verify(m_spark).set(speedCaptor.capture());

      // The speed is negative. See if the current is less than the maximum
      // current (meaning that there should be no speed limiting).
      if(current <= Constants.Intake.kMaxCurrent) {
        // Verify that the speed was unchanged.
        assertTrue(speedCaptor.getValue() == speed);
      } else {
        // See if the speed is positive or negative.
        if(speed < 0) {
          // Verify that the speed is still negative (or zero) and is greater
          // than the target speed (less than the target speed in absolute
          // terms).
          assertTrue(speedCaptor.getValue() <= 0);
          assertTrue(speedCaptor.getValue() > speed);
        } else {
          // Verify that the speed is still positive (or zero) and is less than
          // the target speed.
          assertTrue(speedCaptor.getValue() >= 0);
          assertTrue(speedCaptor.getValue() < speed);
        }
      }
    } else {
      // Verify that the speed of the motor controller was not set.
      verify(m_spark, never()).set(0.0);
    }
  }

  // Generates a test data set for the current limiting test.
  static Stream<Arguments> runLimited() {
    // Create a stream builder for storing all the test data sets.
    Stream.Builder<Arguments> builder = Stream.builder();

    // Loop through a set of speeds.
    for(int i = -10; i <= 10; i++) {
      // Loop through a set of currents.
      for(int j = 1; j <= 25; j++) {
       // Add a data set for this speed and current.
       builder.add(Arguments.of(i / 10.0, j / 1.0));
      }
    }

    // Add a variety of randomized tests.
    for(int i = 0; i < 100; i++) {
      // Add a data set for a random speed and a random current.
      builder.add(Arguments.of((Math.random() * 2) - 1, Math.random() * 25));
    }

    // Create and return an argument stream of these data sets.
    return builder.build();
  }
 
  // A unit test for stopping the intake.
  @Test
  void stop() {
    // Reset the motor controller mock object.
    Mockito.reset(m_spark);

    // Start the intake running.
    m_intake.run(1.0);
    m_intake.periodic();

    // Stop the intake.
    m_intake.stop();
    m_intake.periodic();

    // Verify that the motor controller was stopped.
    verify(m_spark).set(0.0);
  }
}