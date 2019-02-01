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
    public static final int PCM_SOLGRABBER = 22;
    
    public static final int GRABBER_ANALOG_POT = 0;
    public static final int INNER_LIMIT_SWITCH = 10;
    public static final int OUTER_LIMIT_SWITCH  = 11;
    public static final int GRABBER_ENC = 3;
    public static final int CAN_OPEN_GRABBER = 1;

}
