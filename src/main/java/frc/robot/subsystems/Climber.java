/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

//TODO : find out if these motors are brushless or brushed

/**
 * Climber subsystem
 */
public class Climber extends Subsystem {
  private CANSparkMax m_front, m_back;
  private DigitalInput frontLimitSwitch, backLimitSwitch;
  public Climber() {
    m_front = new CANSparkMax(RobotMap.CAN_SPARK_LIFT_FRONT, MotorType.kBrushed);
    m_back = new CANSparkMax(RobotMap.CAN_SPARK_LIFT_BACK, MotorType.kBrushed);

    frontLimitSwitch = new DigitalInput(1); // Currently Arbitrary numbers
    backLimitSwitch = new DigitalInput(2);
  }
  /**
   * Sets the climber motors to the desired speed
   * @param climberSpeed The desired speed for the climber motors
   */
  public void setClimber(double climberSpeed) {
    m_front.set(climberSpeed);
    m_back.set(-climberSpeed);
  }

  /**
   * Stops the climber motors
   */
  public void stopClimber() {
    setClimber(0.0);
  }
  
  /**
   * 
   * @return
   */
  public boolean isClimberTopped() {
    return (frontLimitSwitch.get() || backLimitSwitch.get());
  }

  public double getClimberHeight() {
    return (m_front.getEncoder().getPosition() + m_back.getEncoder().getPosition())/2.0;
  }

  @Override
  public void initDefaultCommand() {
  }
} 
