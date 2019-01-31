/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Class containing all the constants used by the robot
 */
public class RobotConstants {

  public static final int ENC_PPR = 128;

  public static final double MIN_AIM_COMMAND = 0.05;
  public static final double DEFAULT_DEADZONE = 0.1;
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double ENC_DIST_PER_PULSE = Math.PI*WHEEL_DIAMETER / (double) ENC_PPR;
}
