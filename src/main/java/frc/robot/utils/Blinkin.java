// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Spark;

/**
 * A "motor controller" for the Blinkin light controller.
 */
public class Blinkin extends Spark {
  public final static double kRainbow_Rainbow = -0.99;
  public final static double kRainbow_Party = -0.97;
  public final static double kRainbow_Ocean = -0.95;
  public final static double kRainbow_Lava = -0.93;
  public final static double kRainbow_Forest = -0.91;
  public final static double kRainbow_Glitter = -0.89;
  public final static double kConfetti = -0.87;
  public final static double kShot_Red = -0.85;
  public final static double kShot_White = -0.83;
  public final static double kShot_Blue = -0.81;
  public final static double kSinelon_Rainbow = -0.79;
  public final static double kSinelon_Party = -0.77;
  public final static double kSinelon_Ocean = -0.75;
  public final static double kSinelon_Lava = -0.73;
  public final static double kSinelon_Forest = -0.71;
  public final static double kBPM_Rainbow = -0.69; 
  public final static double kBPM_Party = -0.67;
  public final static double kBPM_Ocean = -0.65;
  public final static double kBPM_Lava = -0.63;
  public final static double kBPM_Forest = -0.61;
  public final static double kFire_Medium = -0.59;
  public final static double kFire_Large = -0.57;
  public final static double kTwinkles_Rainbow = -0.55;
  public final static double kTwinkles_Party = -0.53;
  public final static double kTwinkles_Ocean = -0.51;
  public final static double kTwinkles_Lava = -0.49;
  public final static double kTwinkles_Forest = -0.47;
  public final static double kColor_Waves_Rainbow = -0.45;
  public final static double kColor_Waves_Party = -0.43;
  public final static double kColor_Waves_Ocean = -0.41;
  public final static double kColor_Waves_Lava = -0.39;
  public final static double kColor_Waves_Forest = -0.37;
  public final static double kLarson_Scanner_Red = -0.35;
  public final static double kLarson_Scanner_Gray = -0.33;
  public final static double kLight_Chase_Red = -0.31;
  public final static double kLight_Chase_Blue = -0.29;
  public final static double kLight_Chase_Gray = -0.27;
  public final static double kHeartbeat_Red = -0.25;
  public final static double kHeartbeat_White = -0.23;
  public final static double kHeartbeat_Blue = -0.21;
  public final static double kHeartbeat_Gray = -0.19;
  public final static double kBreath_Red = -0.17;
  public final static double kBreath_Blue = -0.15;
  public final static double kBreath_Gray = -0.13;
  public final static double kStrobe_Red = -0.11;
  public final static double kStrobe_Blue = -0.09;
  public final static double kStrobe_Gold = -0.07;
  public final static double kStrobe_White = -0.05;
  public final static double kBlend_To_Black_Color1 = -0.03;
  public final static double kLarson_Scanner_Color1 = -0.01;
  public final static double kLight_Chase_Color1 = 0.01;
  public final static double kHeartbeat_Slow_Color1 = 0.03;
  public final static double kHeartbeat_Medium_Color1 = 0.05;
  public final static double kHeartbeat_Fast_Color1 = 0.07;
  public final static double kBreath_Slow_Color1 = 0.09;
  public final static double kBreath_Fast_Color1 = 0.11;
  public final static double kShot_Color1 = 0.13;
  public final static double kStrobe_Color1 = 0.15;
  public final static double kBlend_To_Black_Color2 = 0.17;
  public final static double kLarson_Scanner_Color2 = 0.19;
  public final static double kLight_Chase_Color2 = 0.21;
  public final static double kHeartbeat_Slow_Color2 = 0.23;
  public final static double kHeartbeat_Medium_Color2 = 0.25;
  public final static double kHeartbeat_Fast_Color2 = 0.27;
  public final static double kBreath_Slow_Color2 = 0.29;
  public final static double kBreath_Fast_Color2 = 0.31;
  public final static double kShot_Color2 = 0.33;
  public final static double kStrobe_Color2 = 0.35;
  public final static double kSparkle_Color1_On_Color2 = 0.37;
  public final static double kSparkle_Color2_On_Color1 = 0.39;
  public final static double kGradient_Color1_Color2 = 0.41;
  public final static double kBPM_Color1_Color2 = 0.43;
  public final static double kEnd_To_End_Color1_Color2 = 0.45;
  public final static double kEnd_To_End_Color2_Color1 = 0.47;
  public final static double kColor1_Color2 = 0.49;
  public final static double kTwinkles_Color1_Color2 = 0.51;
  public final static double kColor_Waves_Color1_Color2 = 0.53;
  public final static double kSinelon_Color1_Color2 = 0.55;
  public final static double kColor_HotPink = 0.57;
  public final static double kColor_DarkRed = 0.59;
  public final static double kColor_Red = 0.61;
  public final static double kColor_RedOrange = 0.63;
  public final static double kColor_Orange = 0.65;
  public final static double kColor_Gold = 0.67;
  public final static double kColor_Yellow = 0.69;
  public final static double kColor_LawnGreen = 0.71;
  public final static double kColor_Lime = 0.73;
  public final static double kColor_DarkGreen = 0.75;
  public final static double kColor_Green = 0.77;
  public final static double kColor_BlueGreen = 0.79;
  public final static double kColor_Aqua = 0.81;
  public final static double kColor_SkyBlue = 0.83;
  public final static double kColor_DarkBlue = 0.85;
  public final static double kColor_Blue = 0.87;
  public final static double kColor_BlueViolet = 0.89;
  public final static double kColor_Violet = 0.91;
  public final static double kColor_White = 0.93;
  public final static double kColor_Gray = 0.95;
  public final static double kColor_DarkGray = 0.97;
  public final static double kColor_Black = 0.99;

  /**
   * Creates a new Blinkin controller.
   *
   * @param channel is the PWM channel to which the Blinkin is connected.
   */
  public Blinkin(int channel) {
    super(channel);
  }

  /**
   * Sets the color/pattern that is displayed by the Blinkin.
   *
   * @param pattern is the color/pattern to display.
   */
  public void setPattern(double pattern) {
    super.set(pattern);
  }

  /**
   * Gets the color/pattern that is being displayed by the Blinkin.
   *
   * @return The color/patter that is being displayed.
   */
  public double getPattern() {
    return super.get();
  }
}