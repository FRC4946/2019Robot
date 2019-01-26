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
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax m_leftFront = new CANSparkMax(RobotMap.CAN_LIFT_LEFT_FRONT, MotorType.kBrushed);
  private CANSparkMax m_leftBack = new CANSparkMax(RobotMap.CAN_LIFT_LEFT_BACK, MotorType.kBrushed);
  private CANSparkMax m_rightFront = new CANSparkMax(RobotMap.CAN_LIFT_RIGHT_FRONT, MotorType.kBrushed);
  private CANSparkMax m_rightBack = new CANSparkMax(RobotMap.CAN_LIFT_RIGHT_BACK, MotorType.kBrushed);

  private SpeedControllerGroup m_liftMotors = new SpeedControllerGroup(m_leftFront, m_leftBack, m_rightFront, m_rightBack);

  public Climber(){

  }

  public void setClimber(double climberSpeed){
    m_liftMotors.set(climberSpeed);
  }

  public void stopClimber() {
    setClimber(0.0);
  }

  @Override
  public void initDefaultCommand() {

  }
}
