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

  // PID
  public static final double LIMELIGHT_TURN_KP = -0.1; // kP for limelight turn
  public static final double LIMELIGHT_TURN_KI = 0.0; // kI for limelight turn
  public static final double LIMELIGHT_TURN_KD = 0.0; // kD for limelight turn

  public static final double LIMELIGHT_DISTANCE_KP = -0.1;
  public static final double LIMELIGHT_DISTANCE_KI = 0.0;
  public static final double LIMELIGHT_DISTANCE_KD = 0.0;

  public static final double CAN_DRIVE_LEFT_FRONT_KP = 0.1;
  public static final double CAN_DRIVE_LEFT_FRONT_KI = 0.0;
  public static final double CAN_DRIVE_LEFT_FRONT_KD = 0.0;

  public static final double CAN_DRIVE_LEFT_BACK_KP = 0.1;
  public static final double CAN_DRIVE_LEFT_BACK_KI = 0.1;
  public static final double CAN_DRIVE_LEFT_BACK_KD = 0.1;

  public static final double CAN_DRIVE_RIGHT_FRONT_KP = 0.1;
  public static final double CAN_DRIVE_RIGHT_FRONT_KI = 0.0;
  public static final double CAN_DRIVE_RIGHT_FRONT_KD = 0.0;

  public static final double CAN_DRIVE_RIGHT_BACK_KP = 0.1;
  public static final double CAN_DRIVE_RIGHT_BACK_KI = 0.0;
  public static final double CAN_DRIVE_RIGHT_BACK_KD = 0.0;

  public static final double CAN_DRIVE_GYRO_TURN_KP = 0.01; // ARE THESE ONES NECESSARY??
  public static final double CAN_DRIVE_GYRO_TURN_KI = 0.0;
  public static final double CAN_DRIVE_GYRO_TURN_KD = 0.0;

  public static final double CAN_DRIVE_KP = 0.1; // ARE THESE ONES NECESSARY??
  public static final double CAN_DRIVE_KI = 0.0;
  public static final double CAN_DRIVE_KD = 0.0;

  public static final double MIN_AIM_COMMAND = 0.05;

  public static final double JOYSTICK_DEADZONE = 0.2;
}
