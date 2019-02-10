/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

 /**
 * Slider subsystem
 */
public class Slider extends Subsystem {

  private TalonSRX m_Left = new TalonSRX(RobotMap.CAN_SLIDER_MOTOR);
  private TalonSRX m_Right = new TalonSRX(RobotMap.CAN_SLIDER_MOTOR);

   /*
   */
  public void runLeft(double speed) {
    m_Left.set(ControlMode.PercentOutput, speed);
  }

   public void runRight(double speed) {
    m_Right.set(ControlMode.PercentOutput, -speed);
    }

  /**
   * Stops the slider
   */
  public void stopAll() {
    m_Right.set(ControlMode.PercentOutput, 0.0);
    m_Left.set(ControlMode.PercentOutput, 0.0);
  }

    @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
