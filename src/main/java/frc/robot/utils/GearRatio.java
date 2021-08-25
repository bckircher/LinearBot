// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

/**
 * Computes the scaling factor used to convert encoder ticks to distance.
 */
public class GearRatio
{
  /**
   * Computes the distance traveled for each encoder tick when driving a wheel.
   *
   * <p>As an example, for a NEO (42 ticks per revolution) with a 16-tooth gear
   * on the output shaft connected to a 80-tooth gear on a shaft with a 6-inch
   * diameter wheel, the call would be:
   * 
   * <pre>compute(42, 16, 80, 6);</pre>
   * 
   * @param dArgs The first argument is the encoder ticks per mechanical
   *              revolution, the last is the wheel diameter, and the pairs
   *              between are the input then output gear tooth counts.
   * 
   * @return The distance per encoder tick.
   */
  public static double
  computeWheel(double... dArgs) {
    double dRatio;
    int iIdx;
    if((dArgs.length < 2) || ((dArgs.length & 1) == 1)) {
      return(0);
    }
    dRatio = (dArgs[dArgs.length - 1] * Math.PI) / dArgs[0];
    for(iIdx = 1; iIdx < (dArgs.length - 1); iIdx += 2) {
      dRatio *= dArgs[iIdx];
      dRatio /= dArgs[iIdx + 1];
    }
    return(dRatio);
  }

  /**
   * Computes the distance traveled for each encoder tick when driving a lead
   * screw.
   *
   * <p>As an example, for a NEO (42 ticks per revolution) with a 16-tooth gear
   * on the output shaft connected to a 32-tooth gear on a shaft with a
   * 0.5-inch per rotation lead screw, the call would be:
   * 
   * <pre>compute(42, 16, 32, 0.5);</pre>
   * 
   * @param dArgs The first argument is the encoder ticks per mechanical
   *              revolution, the last is the lead screw travel per rotation,
   *              and the pairs between are the input then output gear tooth
   *              counts.
   * 
   * @return The distance per encoder tick.
   */
  public static double
  computeLeadScrew(double... dArgs) {
    double dRatio;
    int iIdx;
    if((dArgs.length < 2) || ((dArgs.length & 1) == 1)) {
      return(0);
    }
    dRatio = dArgs[dArgs.length - 1] / dArgs[0];
    for(iIdx = 1; iIdx < (dArgs.length - 1); iIdx += 2) {
      dRatio *= dArgs[iIdx];
      dRatio /= dArgs[iIdx + 1];
    }
    return(dRatio);
  }

  // A unit test for the gear ratio computation function.
  public static void main(String... args)
  {
    double ratio, expected;
    boolean error = false;

    ratio = GearRatio.computeWheel(42, 6);
    expected = 0.448799;
    if(Math.abs(ratio - expected) > 0.00001) {
      System.out.printf("GearRatio.computeWheel(42, 6) returned %f, " +
                        "expecting %f\n", ratio, expected);
      error = true;
    }

    ratio = GearRatio.computeWheel(42, 16, 80, 6);
    expected = 0.08976;
    if(Math.abs(ratio - expected) > 0.00001) {
      System.out.printf("GearRatio.computeWheel(42, 16, 80, 6) returned " +
                        "%f, expecting %f\n", ratio, expected);
      error = true;
    }

    ratio = GearRatio.computeWheel(42, 16, 80, 30, 50, 6);
    expected = 0.053856;
    if(Math.abs(ratio - expected) > 0.00001) {
      System.out.printf("GearRatio.computeWheel(42, 16, 80, 30, 50, 6) " +
                        "returned %f, expecting %f\n", ratio, expected);
      error = true;
    }

    ratio = GearRatio.computeLeadScrew(42, 0.5);
    expected = 0.011905;
    if(Math.abs(ratio - expected) > 0.00001) {
      System.out.printf("GearRatio.computeLeadScrew(42, 0.5) returned %f, " +
                        "expecting %f\n", ratio, expected);
      error = true;
    }

    ratio = GearRatio.computeLeadScrew(42, 16, 32, 1);
    expected = 0.011905;
    if(Math.abs(ratio - expected) > 0.00001) {
      System.out.printf("GearRatio.computeLeadScrew(42, 16, 32, 1) " +
                        "returned %f, expecting %f\n", ratio, expected);
      error = true;
    }

    if(!error) {
      System.out.printf("All tests passed!\n");
    }
  }
}