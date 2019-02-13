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

  public static final double DEFAULT_DEADZONE = 0.05;
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double ENC_DIST_PER_PULSE = Math.PI*WHEEL_DIAMETER / (double) ENC_PPR;

  public static final double ELEVATOR_SCALING_VALUE = 0.0;
  public static final double ELEVATOR_OFFSET_VALUE = 0.0;

  public static final double ELEVATOR_MINIMUM_HEIGHT = 0.0; 
  public static final double ELEVATOR_MAXIMUM_HEIGHT = 0.0;
  public static final double ELEVATOR_INTERFERE_MIN = 0.0;
  public static final double ELEVATOR_INTERFERE_MAX = 0.0;
  public static final double ELEVATOR_SWITCH_HEIGHT = 0.0;
  public static final double ELEVATOR_SCALE_LOWHEIGHT = 0.0;
  public static final double ELEVATOR_SCALE_HIGHHEIGHT = 0.0;
  public static final double ELEVATOR_RUNG_HEIGHT = 0.0;
  
  public static final double UPWARDS_OUTER_INTAKE_POSITION = 0.0;
  public static final double DOWNWARDS_OUTER_INTAKE_POSITION = 600.0;
  public static final double INTAKE_POT_MIN = 0.0;
  public static final double INTAKE_POT_MAX = 1000.0;
  
  public static final double GRABBER_ARM_OUT = 1000.0;
  public static final double GRABBER_ARM_HOLD_HATCH = 900.0;
  public static final double GRABBER_ARM_IN = 0.0;

  public static final double CLIMBER_MIN_HEIGHT = 0.0;
  public static final double CLIMBER_MAX_HEIGHT = 40.0;
  public static final double LOWER_PLATFORM_HEIGHT = 6.0; //inches
  public static final double UPPER_PLATFORM_HEIGHT = 19.0; //inches
  
  // -------------- PID time (default P = 0.002, I = 0.0, D = 0.0)

  public static final double PID_ABS_TURN_P = 0.2;
  public static final double PID_ABS_TURN_I = 0.0;
  public static final double PID_ABS_TURN_D = 0.0;

  public static final double PID_DRIVE_STRAIGHT_P = 0.005875;
  public static final double PID_DRIVE_STRAIGHT_I = 0.00002;
  public static final double PID_DRIVE_STRAIGHT_D = 0.002;

  public static final double PID_DRIVE_STRAIGHT_GYRO_P = 0.2;
  public static final double PID_DRIVE_STRAIGHT_GYRO_I = 0.005;
  public static final double PID_DRIVE_STRAIGHT_GYRO_D = 0.0;
  
  public static final double PID_ROTATE_TO_TARGET_P = 0.02;
  public static final double PID_ROTATE_TO_TARGET_I = 0.0;
  public static final double PID_ROTATE_TO_TARGET_D = 0.0;

  public static final double PID_STRAFE_TO_TARGET_P = 0.004;
  public static final double PID_STRAFE_TO_TARGET_I = 0.0005;
  public static final double PID_STRAFE_TO_TARGET_D = 0.0;

  public static final double PID_STRAFE_TO_TARGET_GYRO_P = 0.018;
  public static final double PID_STRAFE_TO_TARGET_GYRO_I = 0.001;
  public static final double PID_STRAFE_TO_TARGET_GYRO_D = 0.0;

  public static final double PID_TARGET_LINE_P = 0.002;
  public static final double PID_TARGET_LINE_I = 0.0;
  public static final double PID_TARGET_LINE_D = 0.0;

  public static final double PID_TURN_P = 0.00645;
  public static final double PID_TURN_I = 0.000001;
  public static final double PID_TURN_D = 0.002;

  public static final double PID_ELEVATOR_MOVE_TO_HEIGHT_P = 0.02;
  public static final double PID_ELEVATOR_MOVE_TO_HEIGHT_I = 0.0;
  public static final double PID_ELEVATOR_MOVE_TO_HEIGHT_D = 0.0;

  public static final double SLIDER_ACTUATION_TIME = 1.0; //actuation time for the slider in s, dummy number rn

  /* -------------- deprecated time :(

  public static final double MIN_AIM_COMMAND = 0.05;

  */
}
