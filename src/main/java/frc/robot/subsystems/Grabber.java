/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * TODO: Document
 */
public class Grabber extends Subsystem {

  private TalonSRX m_grabberMotor;
  private DigitalInput m_innerSwitch, m_outerSwitch;

  public Grabber() {
    m_grabberMotor = new TalonSRX(RobotMap.CAN_GRABBER_MOTOR);
    m_innerSwitch = new DigitalInput(RobotMap.DIO_GRABBER_IN);
    m_outerSwitch = new DigitalInput(RobotMap.DIO_GRABBER_OUT);
  }

  public void setGrabber(double speed) {
    m_grabberMotor.set(ControlMode.PercentOutput, speed); 
  }

  public boolean getGrabberIn() {
    return m_innerSwitch.get() && !m_outerSwitch.get();
  }

  public boolean getGrabberOut() {
    return m_outerSwitch.get() && !m_innerSwitch.get();
  }

  @Override
  public void initDefaultCommand() {

  }
}
