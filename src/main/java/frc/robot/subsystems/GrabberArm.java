/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class GrabberArm extends Subsystem {
  
  private TalonSRX m_armMotor;  
  private AnalogPotentiometer m_armPot;

  public GrabberArm() {
    m_armMotor = new TalonSRX(RobotMap.CAN_TALON_GRABBER_ARM);
    m_armPot = new AnalogPotentiometer(RobotMap.ANALOG_GRABBER_ARM_POT);
  }

  /**
   * Gets the value of the analog potentiometer
   */
  public double getPot() {
    return m_armPot.get();
  }

  /**
   * Sets the arm's motor to the desired speed
   * @param speed The speed that the motor runs at
   */
  public void setArm(double speed) { 
    m_armMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
  }
}
