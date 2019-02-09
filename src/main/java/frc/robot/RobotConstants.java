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
  public static final double DEFAULT_DEADZONE = 0.05;
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double ENC_DIST_PER_PULSE = Math.PI*WHEEL_DIAMETER / (double) ENC_PPR;

//Same numbers from last year because the real numbers are unknown
  public static final double ELEVATOR_SCALING_VALUE = 127.59;
	public static final double ELEVATOR_OFFSET_VALUE = 0.7349;

  public static final double ELEVATOR_MINIMUM_HEIGHT = 5.5; 
  public static final double ELEVATOR_MAXIMUM_HEIGHT = 96;
  public static final double ELEVATOR_INTERFERE_MIN = 6.25;
	public static final double ELEVATOR_INTERFERE_MAX = 32;
	public static final double ELEVATOR_SWITCH_HEIGHT = 42.0;
	public static final double ELEVATOR_SCALE_LOWHEIGHT = 65.0;
	public static final double ELEVATOR_SCALE_HIGHHEIGHT = 92.0;
  public static final double ELEVATOR_RUNG_HEIGHT = 78.0;
  
  public static final double UPWARDS_OUTER_INTAKE_POSITION = 0.0;
  public static final double DOWNWARDS_OUTER_INTAKE_POSITION = 0.0;
  public static final double INTAKE_POT_MIN = 0.0;
  public static final double INTAKE_POT_MAX = 0.0;
}
