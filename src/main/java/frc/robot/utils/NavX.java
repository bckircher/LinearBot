// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * A wrapper class for the navX that adds some additional functionality.
 */
public class NavX extends AHRS {
  /**
   * Creates a new navX.
   */
  public NavX() {
    super(SPI.Port.kMXP);
  }

  // Adds items from this subsystem to the sendable builder, so that they get
  // sent to the driver station via network tables.
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

  // Override the method for retrieving the yaw from the NavX. What the NavX
  // considers to be a positive yaw is the opposite of what WPILib considers to
  // be a positive yaw, so this handles that difference.
  @Override
  public float getYaw() {
    return -super.getYaw();
  }

  // Override the method for retrieving the angle from the NavX. What the NavX
  // considers to be a positive angle is the opposite of what WPILib considers
  // to be a positive angle, so this handles that difference.
  @Override
  public double getAngle() {
    return -super.getAngle();
  }

  /**
   * Gets the velocity of the robot along its current direction of travel.
   *
   * @return the velocity of the robot.
   */
  private double getVelocity() {
    // Get the X and Y velocity of the robot.
    double x = getVelocityX();
    double y = getVelocityY();

    // Compute and return the Euclidean velocity of the robot.
    return Math.sqrt((x * x) + (y * y));
  }

  /**
   * Gets a rotation matrix representing the robot's current heading.
  *
   * @return the rotation matrix.
   */
  public Rotation2d getRotation2D() {
    return new Rotation2d(Math.toRadians(getYaw()));
  }
}
