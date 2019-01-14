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

public static final int USB_DS_DRIVESTICK = 0;
public static final int USB_DS_OPERATORSTICK = 1;
// Joystick ports

public static final int CAN_DRIVE_FRONT_RIGHT = 0;
public static final int CAN_DRIVE_MID_RIGHT = 1;
public static final int CAN_DRIVE_BACK_RIGHT = 2;
public static final int CAN_DRIVE_FRONT_LEFT = 3;
public static final int CAN_DRIVE_MID_LEFT = 4;
public static final int CAN_DRIVE_BACK_LEFT = 5;
//Drive train motor ports

}
