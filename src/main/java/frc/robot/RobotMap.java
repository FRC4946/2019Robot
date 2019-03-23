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

  public static final int USB_DS_DRIVESTICK = 0;
  public static final int USB_DS_OPERATORSTICK = 1;

  public static final int CAN_SPARK_DRIVE_LEFT_BACK = 1;
  public static final int CAN_SPARK_DRIVE_LEFT_FRONT = 2;
  public static final int CAN_SPARK_DRIVE_RIGHT_FRONT = 3;
  public static final int CAN_SPARK_DRIVE_RIGHT_BACK = 4;
  public static final int CAN_SPARK_LIFT_FRONT = 5;
  public static final int CAN_SPARK_LIFT_BACK = 6;
  public static final int CAN_SPARK_INTAKE_ELBOW = 7;

  public static final int CAN_SPARK_ELEVATOR = 8; 

  public static final int CAN_TALON_GRABBER_MOTOR = 9;
  public static final int CAN_TALON_GRABBER_ARM = 10;
  public static final int CAN_TALON_INTAKE_OUTER = 11;
  public static final int CAN_TALON_INTAKE_INNER_LEFT = 12;
  public static final int CAN_TALON_INTAKE_INNER_RIGHT = 13;

  public static final int DIO_DRIVE_LEFT_FRONT_ENCA = 10;
  public static final int DIO_DRIVE_LEFT_FRONT_ENCB = 11;
  public static final int DIO_DRIVE_LEFT_BACK_ENCA = 2;
  public static final int DIO_DRIVE_LEFT_BACK_ENCB = 3;
  public static final int DIO_DRIVE_RIGHT_FRONT_ENCA = 4;
  public static final int DIO_DRIVE_RIGHT_FRONT_ENCB = 5;
  public static final int DIO_DRIVE_RIGHT_BACK_ENCA = 6;
  public static final int DIO_DRIVE_RIGHT_BACK_ENCB = 7;
  public static final int DIO_GRABBER_IN = 0;
  public static final int DIO_GRABBER_OUT = 1;
  public static final int DIO_CLIMBER_UP_FRONT = 12;
  public static final int DIO_CLIMBER_UP_BACK = 13;

  public static final int ANALOG_ELEVATOR_POT = 0;
  public static final int ANALOG_ELEVATOR_FRONT_POT = 1;
  public static final int ANALOG_GRABBER_ARM_POT = 2;
  public static final int ANALOG_INTAKE_POT = 4;
  public static final int ANALOG_ELEVATOR_BACK_POT = 5;

}
