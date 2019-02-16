/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

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

  public static final double UPWARDS_ELBOW_ANGLE = 90;
  public static final double STARTING_ELBOW_ANGLE = 30;
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

  public static double PID_DRIVE_STRAIGHT_P = 0.005875;
  public static double PID_DRIVE_STRAIGHT_I = 0.00002;
  public static double PID_DRIVE_STRAIGHT_D = 0.002;

  public static double PID_DRIVE_STRAIGHT_GYRO_P = 0.2;
  public static double PID_DRIVE_STRAIGHT_GYRO_I = 0.005;
  public static double PID_DRIVE_STRAIGHT_GYRO_D = 0.0;

  public static double PID_ROTATE_TO_TARGET_P = 0.02;
  public static double PID_ROTATE_TO_TARGET_I = 0.0;
  public static double PID_ROTATE_TO_TARGET_D = 0.0;

  public static double PID_STRAFE_TO_TARGET_P = 0.004;
  public static double PID_STRAFE_TO_TARGET_I = 0.0005;
  public static double PID_STRAFE_TO_TARGET_D = 0.0;

  public static double PID_STRAFE_TO_TARGET_GYRO_P = 0.018;
  public static double PID_STRAFE_TO_TARGET_GYRO_I = 0.001;
  public static double PID_STRAFE_TO_TARGET_GYRO_D = 0.0;

  public static double PID_TARGET_LINE_P = 0.002;
  public static double PID_TARGET_LINE_I = 0.0;
  public static double PID_TARGET_LINE_D = 0.0;

  public static double PID_TURN_P = 0.00645;
  public static double PID_TURN_I = 0.000001;
  public static double PID_TURN_D = 0.002;

  public static double PID_ELEVATOR_MOVE_TO_HEIGHT_P = 0.02;
  public static double PID_ELEVATOR_MOVE_TO_HEIGHT_I = 0.0;
  public static double PID_ELEVATOR_MOVE_TO_HEIGHT_D = 0.0;

  /* -------------- deprecated time :(

  public static final double MIN_AIM_COMMAND = 0.05;

  */

  public static void loadPrefs(Preferences prefs) {
		PID_DRIVE_STRAIGHT_P = prefs.getDouble("Drive Straight P", 0.005875);
		PID_DRIVE_STRAIGHT_I = prefs.getDouble("Drive Straight I", 0.00002);
		PID_DRIVE_STRAIGHT_D = prefs.getDouble("Drive Straight D", 0.002);

    PID_DRIVE_STRAIGHT_GYRO_P = prefs.getDouble("Drive Straight Gyro P", 0.2);
    PID_DRIVE_STRAIGHT_GYRO_I = prefs.getDouble("Drive Straight Gyro I", 0.005);
    PID_DRIVE_STRAIGHT_GYRO_D = prefs.getDouble("Drive Straight Gyro D", 0.0);

    PID_TURN_P = prefs.getDouble("Turn P", 0.00645);
    PID_TURN_I = prefs.getDouble("Turn I", 0.000001);
    PID_TURN_D = prefs.getDouble("Turn D", 0.002);

    PID_TARGET_LINE_P = prefs.getDouble("Target Line P", 0.002);
    PID_TARGET_LINE_I = prefs.getDouble("Target Line I", 0.0);
    PID_TARGET_LINE_D = prefs.getDouble("Target Line D", 0.0);

    PID_STRAFE_TO_TARGET_P = prefs.getDouble("Strafe To Target P", 0.004);
    PID_STRAFE_TO_TARGET_I = prefs.getDouble("Strafe To Target I", 0.0005);
    PID_STRAFE_TO_TARGET_D = prefs.getDouble("Strafe To Target D", 0.0);

    PID_STRAFE_TO_TARGET_GYRO_P = prefs.getDouble("Strafe To Target Gyro P", 0.018);
    PID_STRAFE_TO_TARGET_GYRO_I = prefs.getDouble("Strafe To Target Gyro I", 0.001);
    PID_STRAFE_TO_TARGET_GYRO_D = prefs.getDouble("Strafe To Target Gyro D", 0.0);

    PID_ELEVATOR_MOVE_TO_HEIGHT_P = prefs.getDouble("Elevator P", 0.02);
    PID_ELEVATOR_MOVE_TO_HEIGHT_I = prefs.getDouble("Elevator I", 0.0);
    PID_ELEVATOR_MOVE_TO_HEIGHT_D = prefs.getDouble("Elevator D", 0.0);
  }

  public static void  repopulatePrefs(Preferences prefs) {
		prefs.putDouble("Drive Straight P", PID_DRIVE_STRAIGHT_P);
		prefs.putDouble("Drive Straight I", PID_DRIVE_STRAIGHT_I);
    prefs.putDouble("Drive Straight D", PID_DRIVE_STRAIGHT_D);

    prefs.putDouble("Drive Straight Gyro P", PID_DRIVE_STRAIGHT_GYRO_P);
		prefs.putDouble("Drive Straight Gyro I", PID_DRIVE_STRAIGHT_GYRO_I);
    prefs.putDouble("Drive Straight Gyro D", PID_DRIVE_STRAIGHT_GYRO_D);
    
    prefs.putDouble("Turn P", PID_TURN_P);
    prefs.putDouble("Turn I", PID_TURN_I);
    prefs.putDouble("Turn D", PID_TURN_D);

    prefs.putDouble("Target Line P", PID_TARGET_LINE_P);
    prefs.putDouble("Target Line I", PID_TARGET_LINE_I);
    prefs.putDouble("Target Line D", PID_TARGET_LINE_D);

    prefs.putDouble("Strafe To Target P", PID_STRAFE_TO_TARGET_P);
    prefs.putDouble("Strafe To Target I", PID_STRAFE_TO_TARGET_I);
    prefs.putDouble("Strafe To Target D", PID_STRAFE_TO_TARGET_D);

    prefs.putDouble("Strafe To Target Gyro P", PID_STRAFE_TO_TARGET_GYRO_P);
    prefs.putDouble("Strafe To Target Gyro I", PID_STRAFE_TO_TARGET_GYRO_I);
    prefs.putDouble("Strafe To Target Gyro D", PID_STRAFE_TO_TARGET_GYRO_D);

    prefs.putDouble("Elevator P", PID_ELEVATOR_MOVE_TO_HEIGHT_P);
    prefs.putDouble("Elevator I", PID_ELEVATOR_MOVE_TO_HEIGHT_I);
    prefs.putDouble("Elevator D", PID_ELEVATOR_MOVE_TO_HEIGHT_D);
  }

  public static void updatePrefs(Preferences prefs) {
    loadPrefs(prefs);
    repopulatePrefs(prefs);
  }
}
