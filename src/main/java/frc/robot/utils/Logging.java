// Copyright (c) 2021 FRC Team 2881 - The Lady Cans
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.utils;

/**
 * This class provides methods for logging messages to the RioLog as the robot
 * starts, the robot mode changes, and as commands start and end.
 */
public final class Logging {
  /**
   * The time that the robot code started.
   */
  private static final long startTime = System.currentTimeMillis();

  /**
   * Formats a message and writes it to the RioLog.
   *
   * @param message contains the text of the message to be logged.
   */
  public static void log(String message) {
    long time = System.currentTimeMillis() - startTime;
    System.out.printf("[%6.%02f] %s\n", time, message);
  }

  /**
   * Logs the start of the robot code.
   */
  public static void logStart() {
    log("***** Code Start, version " + "blah" + " *****");
  }

  /**
   * Logs a change in the robot mode.
   *
   * @param mode is the new robot mode.
   */
  public static void logMode(String mode) {
    log(">>>>> Robot mode: " + mode + " <<<<<");
  }

  /**
   * Logs the start of a command.
   *
   * @param command is the name of the command class.
   */
  public static void logInit(String command) {
    log("Start command: " + command);
  }

  /**
   * Logs the end of a command.
   *
   * @param command is the name of the command class.
   *
   * @param interrupted is <b>true</b> if the command was interrupted.
   */
  public static void logEnd(String command, boolean interrupted) {
    if(interrupted) {
      log("Interrupted command: " + command);
    } else {
      log("End command: " + command);
    }
  }
}
