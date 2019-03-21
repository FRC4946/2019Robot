/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;
import frc.robot.commands.climber.LiftRobot;;

/**
 * Climber subsystem
 */
public class Climber extends Subsystem {

  private CANSparkMax m_front, m_back;
  private AnalogPotentiometer m_frontPot, m_backPot;

  public Climber() {

    m_front = new CANSparkMax(RobotMap.CAN_SPARK_LIFT_FRONT, MotorType.kBrushless);
    m_back = new CANSparkMax(RobotMap.CAN_SPARK_LIFT_BACK, MotorType.kBrushless);
    m_back.setInverted(true);

    m_backPot = new AnalogPotentiometer(RobotMap.ANALOG_ELEVATOR_BACK_POT, RobotConstants.CLIMBER_SCALING_VALUE, RobotConstants.CLIMBER_OFFSET);
    m_frontPot = new AnalogPotentiometer(RobotMap.ANALOG_ELEVATOR_FRONT_POT);
  }

  /**
   * Sets the climber motors to the desired speed
   * @param climberSpeed The desired speed for the climber motors
   */
  public void setClimber(double climberSpeed) { //negative is down
    if (climberSpeed > 0 && frontIsTopped()) {
      m_front.set(0.0);
    } else {
      m_front.set(climberSpeed);
    }

    if (climberSpeed > 0 && backIsTopped()) {
      m_back.set(0.0);
    } else {
      m_back.set(climberSpeed);
    }
  }

  public void setFront(double climberSpeed) {
    if (climberSpeed > 0 && frontIsTopped()) {
      m_front.set(0.0);
    } else {
      m_front.set(climberSpeed);
    }
  }

  public void setBack(double climberSpeed) {
    if (climberSpeed > 0 && backIsTopped()) {
      m_back.set(0.0);
    } else {
      m_back.set(climberSpeed);
    }
  }

  /**
   * Stops the climber motors
   */
  public void stopClimber() {
    setClimber(0.0);
  }
  
  public AnalogPotentiometer getFrontPot(){
    return m_frontPot;
  }

  public AnalogPotentiometer getBackPot(){
    return m_backPot;  
  }

  /**
   * Tells if the climber has reached the maximum height
   * @return if the climber has reached the maximum height
   */
  public boolean isClimberTopped() {
    return frontIsTopped() || backIsTopped();  
  }

  public boolean frontIsTopped() {
    return getFrontClimberHeight() >= RobotConstants.CLIMBER_MAX_HEIGHT;
  }

  public boolean backIsTopped() {
    return getBackClimberHeight() >= RobotConstants.CLIMBER_MAX_HEIGHT;
  }

  public double getFrontClimberHeight() {
    return m_frontPot.get();
  }

  public double getBackClimberHeight() {
    return -m_backPot.get();
  }

  public CANEncoder getFrontEncoder() {
    return m_front.getEncoder();
  }

  public CANEncoder getBackEncoder() { 
    return m_back.getEncoder();
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
    setDefaultCommand(new LiftRobot()); 
  }
} 
