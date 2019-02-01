/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class GrabberLimitSwitch extends Subsystem {
  private DigitalInput m_outerLimitSwitch;
  private DigitalInput m_innerLimitSwitch;
  private CANSparkMax m_clawMotor;

  public void GrabberLimitSwitch(){
    m_outerLimitSwitch = new DigitalInput (RobotMap.OUTER_LIMIT_SWITCH);
    m_innerLimitSwitch = new DigitalInput (RobotMap.INNER_LIMIT_SWITCH);

    m_clawMotor = new CANSparkMax(RobotMap.CAN_OPEN_GRABBER, MotorType.kBrushless);
  }

  public void innerSwitch (){
    if (m_innerLimitSwitch.get() == true){
      m_clawMotor.set(0);
    }
  }

  public void outerSwitch (){
    if (m_outerLimitSwitch.get() == true){
      m_clawMotor.set(0);
    }
  }

  @Override
  public void initDefaultCommand() {
  
  }
}
