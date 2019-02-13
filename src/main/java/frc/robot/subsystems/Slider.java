/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

 /**
 * Slider subsystem
 */
public class Slider extends Subsystem {

  private TalonSRX m_sliderMotor = new TalonSRX(RobotMap.CAN_TALON_SLIDER_MOTOR);
  private boolean m_sliderOut = false; //set to true if the slider starts out

  /**
   * Sets the speed of the motor on the slider
   * @param speed speed of the motor from -1 to 1
   */
  public void set(double speed) {
    m_sliderMotor.set(ControlMode.PercentOutput, speed);
  }


  /**
   * Gets the state  of the slider mechanism
   * @return true if the slider is out
   */
  public boolean getSliderOut() {
    return m_sliderOut;
  }

  /**
   * Updates the subsystem with the current state of the slider mechanism
   * @param state true if the slider is currently out, false otherwise
   */
  public void setSliderOut(boolean state) {
    m_sliderOut = state;
  }

  /**
   * Stops the slider
   */
  public void stop() {
    m_sliderMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
