/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Utility functions
 */
public class Utilities {

  public double conformAngle(double angle) {

    while (angle < 0) {
      angle += 360;
    }

    return angle % 360; // if angle returns greater than 360
  }

  public double deadzone(double input) {
    return deadzone(input, RobotConstants.DEFAULT_DEADZONE);
  }

  public double deadzone(double input, double deadzone) {
    return Math.abs(input) < Math.abs(deadzone) ? 0 : input;
  }

}
