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
import frc.robot.RobotConstants;
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
  public double getPos() {
    return RobotConstants.GRABBER_ARM_OUT - m_armPot.get();
  }

  /**
   * Sets the arm's motor to the desired speed
   * @param speed The speed that the motor runs at
   */
  public void setArm(double speed) {
    //if(getPos() <= RobotConstants.GRABBER_ARM_IN && speed > 0
      //|| getPos() >= RobotConstants.GRABBER_ARM_OUT && speed < 0) {
      //m_armMotor.set(ControlMode.PercentOutput, 0);
    //} else {
      m_armMotor.set(ControlMode.PercentOutput, speed);
    //}
  }

  public void stop() {
    setArm(0);
  }

  @Override
  public void initDefaultCommand() {
  }
}
