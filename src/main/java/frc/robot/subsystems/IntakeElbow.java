/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeElbow extends Subsystem {

  private CANSparkMax m_outerElbow; 
  private AnalogPotentiometer m_outerIntakePot;
  private boolean m_elbowIsUp;

  public IntakeElbow() {
    m_outerElbow = new CANSparkMax(RobotMap.CAN_INTAKE_OUTER_ELBOW, MotorType.kBrushless);
    m_outerIntakePot = new AnalogPotentiometer(RobotMap.ANALOG_INTAKE_POT);
    m_elbowIsUp = false;
  }

  public double getPot() {
    return m_outerIntakePot.get();
  }

  public void setElbow(double speed) {
    m_outerElbow.set(speed);
  }

  public void setElbowIsUp(boolean isUp) { //is this useful or is this garbage - zheng
    m_elbowIsUp = isUp;
  }

  public boolean getElbowIsUp() {
    return m_elbowIsUp;
  }

  @Override
  public void initDefaultCommand() {

  }
}

