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

  // Joystick ports
  public static final int USB_DS_DRIVESTICK = 0;
  public static final int USB_DS_OPERATORSTICK = 1;

  // Drive train motor ports
  public static final int CAN_DRIVE_LEFT_FRONT = 1;
  public static final int CAN_DRIVE_LEFT_BACK = 0;
  public static final int CAN_DRIVE_RIGHT_FRONT = 2;
  public static final int CAN_DRIVE_RIGHT_BACK = 3;

  public static final int CAN_LIFT_FRONT = 4;
  public static final int CAN_LIFT_BACK = 5;

  // Intake motor ports
  public static final int CAN_INTAKE_OUTER = 6;
  public static final int CAN_INTAKE_OUTER_ELBOW = 7;
  public static final int CAN_INTAKE_INNER_LEFT = 8;
  public static final int CAN_INTAKE_INNER_RIGHT = 9;

  //Grabber
  public static final int CAN_GRABBER_MOTOR = 9;
  public static final int CAN_GRABBER_ARM = 10;
  public static final int DIO_GRABBER_IN = 10;
  public static final int DIO_GRABBER_OUT = 11;
  public static final int ANALOG_GRABBER_POT = 2;

  // Elevator motor ports (dummy ports used)
  public static final int CAN_RUN_ELEVATOR = 0; 

  // Drive train encoder ports
  public static final int DIO_DRIVE_LEFT_FRONT_ENCA = 0;
  public static final int DIO_DRIVE_LEFT_FRONT_ENCB = 1;
  public static final int DIO_DRIVE_LEFT_BACK_ENCA = 2;
  public static final int DIO_DRIVE_LEFT_BACK_ENCB = 3;
  public static final int DIO_DRIVE_RIGHT_FRONT_ENCA = 4;
  public static final int DIO_DRIVE_RIGHT_FRONT_ENCB = 5;
  public static final int DIO_DRIVE_RIGHT_BACK_ENCA = 6;
  public static final int DIO_DRIVE_RIGHT_BACK_ENCB = 7;

  // Intake sensor ports
  public static final int DIO_INTAKE_BANNER_SENSOR = 8;

  // dummy port used
  public static final int GRABBER_ANALOG_POT = 0;
  public static final int INNER_LIMIT_SWITCH = 10;
  public static final int OUTER_LIMIT_SWITCH  = 11;

  // Intake sensor ports
  public static final int INTAKE_BANNER_SENSOR = 0;
  public static final int PCM_SOLGRABBER = 0;
  public static final int PCM_ELEVATOR_BREAK = 1;

  // Elevator potentiometer (Dummy ports used)
  public static final int ANALOG_ELEVATOR_POT = 1;
  public static final int ANALOG_INTAKE_POT = 2;
}
