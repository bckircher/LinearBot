// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class NavX extends AHRS {
  public NavX() {
    super(SPI.Port.kMXP);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Roll", this::getRoll, null);
    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Yaw", this::getYaw, null);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addDoubleProperty("VelocityX", this::getVelocityX, null);
    builder.addDoubleProperty("VelocityY", this::getVelocityY, null);
  }

  private double getVelocity() {
    double x = getVelocityX();
    double y = getVelocityY();
    return(Math.sqrt((x * x) + (y * y)));
  }

  public Rotation2d getRotation2D() {
    return new Rotation2d(getYaw());
  }
}
