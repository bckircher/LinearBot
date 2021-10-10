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
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

// JUnit test of the Elevator subsystem class.
class testElevator {
  // Create a mock of the left motor controller.
  private final static CANSparkMax m_left = mock(CANSparkMax.class);

  // Create a mock of the right motor controller.
  private final static CANSparkMax m_right = mock(CANSparkMax.class);

  // Create a mock of the left encoder.
  private final static CANEncoder m_leftEncoder = mock(CANEncoder.class);

  // Create a mock of the right encoder.
  private final static CANEncoder m_rightEncoder = mock(CANEncoder.class);

  // Create an instance of the Elevator subsystem.
  private final static Elevator m_elevator =
    new Elevator(m_left, m_right, m_leftEncoder, m_rightEncoder);

  @ParameterizedTest
  @MethodSource
  void runEqual(double speed) {
    Mockito.reset(m_left);
    Mockito.reset(m_right);

    when(m_leftEncoder.getPosition()).thenReturn(0.0);
    when(m_rightEncoder.getPosition()).thenReturn(0.0);

    m_elevator.run(speed);

    verify(m_left).set(speed);
    verify(m_right).set(speed);
  }

  static Stream<Arguments> runEqual() {
    Stream.Builder<Arguments> builder = Stream.builder();

    for(int i = -10; i <= 10; i++) {
      builder.add(Arguments.of(i / 20.0));
    }

    for(int i = 0; i < 100; i++) {
      builder.add(Arguments.of(Math.random() - 0.5));
    }

    return builder.build();
  }

  @ParameterizedTest
  @MethodSource
  void runImbalanced(double speed, double left, double right) {
    ArgumentCaptor<Double> leftCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rightCaptor = ArgumentCaptor.forClass(Double.class);

    Mockito.reset(m_left);
    Mockito.reset(m_right);

    when(m_leftEncoder.getPosition()).thenReturn(left);
    when(m_rightEncoder.getPosition()).thenReturn(right);

    m_elevator.run(speed);

    verify(m_left).set(leftCaptor.capture());
    verify(m_right).set(rightCaptor.capture());

    if(speed < 0) {
      if(left < right) {
        assertTrue(leftCaptor.getValue() >= speed);
        assertTrue(Math.abs(rightCaptor.getValue() - speed) < 0.00001);
      } else {
        assertTrue(Math.abs(leftCaptor.getValue() - speed) < 0.00001);
        assertTrue(rightCaptor.getValue() >= speed);
      }
    } else {
      if(left < right) {
        assertTrue(Math.abs(leftCaptor.getValue() - speed) < 0.00001);
        assertTrue(rightCaptor.getValue() <= speed);
      } else {
        assertTrue(leftCaptor.getValue() <= speed);
        assertTrue(Math.abs(rightCaptor.getValue() - speed) < 0.00001);
      }
    }
  }

  static Stream<Arguments> runImbalanced() {
    Stream.Builder<Arguments> builder = Stream.builder();

    for(int i = -10; i <= 10; i++) {
      for(int j = -10; j <= 10; j++) {
        if(j == 0) {
          continue;
        }
        builder.add(Arguments.of(i / 20.0, 0, j / 100.0));
        builder.add(Arguments.of(i / 20.0, j / 100.0, 0));
      }
    }

    for(int i = 0; i < 100; i++) {
      builder.add(Arguments.of(Math.random() - 0.5, (Math.random() * 2) - 1,
                               (Math.random() * 2) - 1));
    }

    return builder.build();
  }

  @ParameterizedTest
  @MethodSource
  void runIndividual(double left, double right) {
    Mockito.reset(m_left);
    Mockito.reset(m_right);

    m_elevator.run(left, right);

    verify(m_left).set(left);
    verify(m_right).set(right);
  }

  static Stream<Arguments> runIndividual() {
    Stream.Builder<Arguments> builder = Stream.builder();

    for(int i = -10; i <= 10; i++) {
      for(int j = -10; j <= 10; j++) {
        builder.add(Arguments.of(i / 10.0, j / 10.0));
      }
    }

    return builder.build();
  }

  @Test
  void stop() {
    Mockito.reset(m_left);
    Mockito.reset(m_right);

    m_elevator.stop();

    verify(m_left).stopMotor();
    verify(m_right).stopMotor();
  }

  @Test
  void reset() {
    Mockito.reset(m_leftEncoder);
    Mockito.reset(m_rightEncoder);

    m_elevator.resetPosition();

    verify(m_leftEncoder).setPosition(0);
    verify(m_rightEncoder).setPosition(0);
  }

  @ParameterizedTest
  @MethodSource
  void getPosition(double left, double right) {
    when(m_leftEncoder.getPosition()).thenReturn(left);
    when(m_rightEncoder.getPosition()).thenReturn(right);

    assertTrue(Math.abs(((left + right) / 2) - m_elevator.getPosition()) <
               0.0001);
  }

  static Stream<Arguments> getPosition() {
    Stream.Builder<Arguments> builder = Stream.builder();

    for(int i = -170; i <= 170; i += 17) {
      for(int j = -170; j <= 170; j += 17) {
        builder.add(Arguments.of(i / 100.0, j / 100.0));
      }
    }

    for(int i = 0; i < 100; i++) {
      builder.add(Arguments.of((Math.random() * 10) - 5,
                               (Math.random() * 10) - 5));
    }

    return builder.build();
  }
}
