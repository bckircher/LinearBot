// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

import java.util.stream.Stream;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

import frc.robot.utils.NavX;

public class testDrive {
  private final NavX m_navX = mock(NavX.class);
  private final CANSparkMax m_leftFront = mock(CANSparkMax.class);
  private final CANSparkMax m_leftBack = mock(CANSparkMax.class);
  private final CANSparkMax m_rightFront = mock(CANSparkMax.class);
  private final CANSparkMax m_rightBack = mock(CANSparkMax.class);
  private final CANEncoder m_leftEncoder = mock(CANEncoder.class);
  private final CANEncoder m_rightEncoder = mock(CANEncoder.class);
  private final Drive m_drive = new Drive(m_navX, m_leftFront, m_leftBack,
                                          m_rightFront, m_rightBack,
                                          m_leftEncoder, m_rightEncoder);

  @ParameterizedTest
  @MethodSource
  void driveArcade(double speed, double rotation, double left, double right) {
    // Create a captor for saving the values that are passed to the motor
    // controllers.
    ArgumentCaptor<Double> leftCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rightCaptor = ArgumentCaptor.forClass(Double.class);

    // Reset the mocks for the motor controllers.
    Mockito.reset(m_leftFront);
    Mockito.reset(m_leftBack);
    Mockito.reset(m_rightFront);
    Mockito.reset(m_rightBack);

    // Run the drive in arcade mode.
    m_drive.driveArcade(speed, rotation);

    // Verify that the appropriate motor controllers were set, as well as that
    // the followers were not set (since they are handled via following).
    verify(m_leftFront).set(leftCaptor.capture());
    verify(m_leftBack, never()).set(0);
    verify(m_rightFront).set(rightCaptor.capture());
    verify(m_rightBack, never()).set(0);

    System.out.println(speed + "," + rotation + "," + left + "," + right + "," + leftCaptor.getValue() + "," + rightCaptor.getValue());
    // Verify that the speeds of the two motors were set correctly.
    assertTrue(Math.abs(left - leftCaptor.getValue()) < 0.00001);
    assertTrue(Math.abs(right - rightCaptor.getValue()) < 0.00001);
  }

  static Stream<Arguments> driveArcade() {
    Stream.Builder<Arguments> builder = Stream.builder();
    double left, right;

    for(int i = -10; i <= 10; i++) {
      builder.add(Arguments.of(i / 10.0, 0.0, Math.copySign(i * i, i) / 100.0,
                               -Math.copySign(i * i, i) / 100.0));
    }

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

    // 0.0 -> 1.0
    // 0.1 -> 0.99
    // 0.2 -> 0.96
    // 0.3 -> 0.91
    // 0.4 -> 0.84
    // 0.5 -> 0.75
    // 0.6 -> 0.64
    // 0.7 -> 0.51
    // 0.8 -> 0.36
    // 0.9 -> 0.19
    // 1.0 -> 0.0

    return builder.build();
  }

  // driveArcade
  // driveTank
  // driveTankVolts
  // stop
  // resetPosition
  // getPosition
  // getWheelSpeeds
  // getPose
  // resetOdometry
}