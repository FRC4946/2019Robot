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
  public static final double LIMELIGHT_TURN_KP = -0.1; // kP for limelight turn
  public static final double LIMELIGHT_TURN_KI = 0.0; // kI for limelight turn
  public static final double LIMELIGHT_TURN_KD = 0.0; // kD for limelight turn

  public static final double LIMELIGHT_DISTANCE_KP = -0.1;
  public static final double LIMELIGHT_DISTANCE_KI = 0.0;
  public static final double LIMELIGHT_DISTANCE_KD = 0.0;

  public static final double MIN_AIM_COMMAND = 0.05;
}
