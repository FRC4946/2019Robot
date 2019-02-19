/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.climber.SetClimberHeight;

//TODO : find out if these motors are brushless or brushed

/**
 * Climber subsystem
 */
public class Climber extends Subsystem {
  private CANSparkMax m_front, m_back;
  private DigitalInput frontLimitSwitch, backLimitSwitch;
  public Climber() {
    m_front = new CANSparkMax(RobotMap.CAN_SPARK_LIFT_FRONT, MotorType.kBrushless);
    m_back = new CANSparkMax(RobotMap.CAN_SPARK_LIFT_BACK, MotorType.kBrushless);
    m_back.setInverted(true);

    frontLimitSwitch = new DigitalInput(RobotMap.DIO_CLIMBER_UP_FRONT);
    backLimitSwitch = new DigitalInput(RobotMap.DIO_CLIMBER_UP_BACK);
  }
  /**
   * Sets the climber motors to the desired speed
   * @param climberSpeed The desired speed for the climber motors
   */
  public void setClimber(double climberSpeed) { //negative is down
    if (climberSpeed < 0 && isClimberTopped()) {
      stopClimber();
    } else {
      m_front.set(climberSpeed);
      m_back.set(climberSpeed);
    }
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
    return (!frontLimitSwitch.get() || !backLimitSwitch.get());
  }

  public double getFrontClimberHeight() {
    return m_front.getEncoder().getPosition();
  }

  public double getBackClimberHeight() {
    return m_back.getEncoder().getPosition();
  }

  public CANPIDController getFrontPIDController() {
    return m_front.getPIDController();
  }

  public CANPIDController getBackPIDController() {
    return m_back.getPIDController();
  }

  public void setFrontPIDController(double p, double i, double d) {    
    m_front.getPIDController().setP(p);
    m_front.getPIDController().setI(i);
    m_front.getPIDController().setD(d);
  }

  public void setBackPIDController(double p, double i, double d) {
    m_back.getPIDController().setP(p);
    m_back.getPIDController().setI(i);
    m_back.getPIDController().setD(d);
  }

  public void resetEncs() {
    m_front.setEncPosition(0);
    m_back.setEncPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new HoldPosition());
  }
} 
