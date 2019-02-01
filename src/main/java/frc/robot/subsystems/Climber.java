/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

//TODO : find out if these motors are brushless or brushed

/**
 * Climber subsystem
 */
public class Climber extends Subsystem {

  private CANSparkMax m_front = new CANSparkMax(RobotMap.CAN_LIFT_FRONT, MotorType.kBrushed);
  private CANSparkMax m_back = new CANSparkMax(RobotMap.CAN_LIFT_BACK, MotorType.kBrushed);

  //private SpeedControllerGroup m_liftMotors = new SpeedControllerGroup(m_front, m_back);

  public void setClimber(double climberSpeed) {
    m_front.set(climberSpeed);
    m_front.set(-climberSpeed);
  }

  public void stopClimber() {
    setClimber(0.0);
  }

  @Override
  public void initDefaultCommand() {
  }
}
