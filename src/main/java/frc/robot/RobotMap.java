/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // Joystick ports
  public static final int USB_DS_DRIVESTICK = 0;
  public static final int USB_DS_OPERATORSTICK = 1;

  // Drive train motor ports
  public static final int CAN_DRIVE_LEFT_FRONT = 1;
  public static final int CAN_DRIVE_LEFT_BACK = 0;
  public static final int CAN_DRIVE_RIGHT_FRONT = 2;
  public static final int CAN_DRIVE_RIGHT_BACK = 3;

  public static final int CAN_LIFT_RIGHT_FRONT = 4;
  public static final int CAN_LIFT_RIGHT_BACK = 5;
  public static final int CAN_LIFT_LEFT_FRONT = 6;
  public static final int CAN_LIFT_LEFT_BACK = 7;

  // Intake motor ports
  public static final int CAN_INTAKE_OUTER_LEFT = 6;
  public static final int CAN_INTAKE_OUTER_RIGHT = 7;
  public static final int CAN_INTAKE_INNER_LEFT = 8;
  public static final int CAN_INTAKE_INNER_RIGHT = 9;

  // Drive train encoder ports
  public static final int CAN_DRIVE_LEFT_FRONT_ENCA = 0;
  public static final int CAN_DRIVE_LEFT_FRONT_ENCB = 1;
  public static final int CAN_DRIVE_LEFT_BACK_ENCA = 6;
  public static final int CAN_DRIVE_LEFT_BACK_ENCB = 7;
  public static final int CAN_DRIVE_RIGHT_FRONT_ENCA = 4;
  public static final int CAN_DRIVE_RIGHT_FRONT_ENCB = 5;
  public static final int CAN_DRIVE_RIGHT_BACK_ENCA = 2;
  public static final int CAN_DRIVE_RIGHT_BACK_ENCB = 3;

  // dummy port used
  public static final int PCM_SOLGRABBER = 0;

  // Intake sensor ports
  public static final int INTAKE_BANNER_SENSOR = 0;
}
