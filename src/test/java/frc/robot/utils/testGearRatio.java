// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

// JUnit test of the GearRatio class.
public class testGearRatio {
  // Tests computing the gear ratio of a direct-driven wheel.
  @Test
  void wheelNoGears() {
    double result = GearRatio.computeWheel(1, 6);
    double expected = 18.849556;
    double error = 0.00001;

    assertTrue(Math.abs(result - expected) < error);
  }

  // Tests computing the gear ratio of a single reduction stage on a wheel.
  @Test
  void wheelOneReduction() {
    double result = GearRatio.computeWheel(1, 16, 80, 6);
    double expected = 3.769911;
    double error = 0.00001;

    assertTrue(Math.abs(result - expected) < error);
  }

  // Tests computing the gear ratio of two reduction stages on a wheel.
  @Test
  void wheelTwoReductions() {
    double result = GearRatio.computeWheel(1, 16, 80, 30, 50, 6);
    double expected = 2.261947;
    double error = 0.00001;

    assertTrue(Math.abs(result - expected) < error);
  }

  // Tests computing the gear ratio of a direct-driven lead-screw.
  @Test
  void leadScrewNoGears() {
    double result = GearRatio.computeLeadScrew(1, 0.5);
    double expected = 0.5;
    double error = 0.00001;

    assertTrue(Math.abs(result - expected) < error);
  }

  // Tests computing the gear ratio of two reduction stages on a lead-screw.
  @Test
  void leadScrewTwoReductions() {
    double result = GearRatio.computeLeadScrew(1, 16, 32, 2, 1, 0.5);
    double expected = 0.5;
    double error = 0.00001;

    assertTrue(Math.abs(result - expected) < error);
  }
}
