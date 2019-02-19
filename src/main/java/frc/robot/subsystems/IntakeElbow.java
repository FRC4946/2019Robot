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
import frc.robot.RobotConstants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeElbow extends Subsystem {

  private CANSparkMax m_outerElbow; 
  private AnalogPotentiometer m_outerIntakePot;

  public IntakeElbow() {
    m_outerElbow = new CANSparkMax(RobotMap.CAN_SPARK_INTAKE_ELBOW, MotorType.kBrushless);
    m_outerIntakePot = new AnalogPotentiometer(RobotMap.ANALOG_INTAKE_POT, RobotConstants.INTAKE_POT_SCALING, RobotConstants.INTAKE_POT_OFFSET); //scale value of 3600 because this is a 10 turn pot
  }


  public double getPos() {
    return m_outerIntakePot.get();
  }

  public void setElbow(double speed) {
    m_outerElbow.set(speed);
  }

  public void stop() {
    m_outerElbow.set(0);
  }

  /**
   * Returns the elbow position
   * @return True if the intake elbow is in the down position 
   */
  public boolean isDown() {
    return getPos() > RobotConstants.INTAKE_POT_DOWN; //within 2 degrees of 0
  }

  /**
   * Returns the elbow position
   * @return True if the intake elbow is in the up position
   */
  public boolean isUp() {
    return getPos() < (RobotConstants.INTAKE_POT_UP-2); //within 2 degrees of upper height
  }

  public boolean isAtBallHeight() {
    return Math.abs(getPos() - RobotConstants.INTAKE_POT_BALL_HEIGHT) < 2;
  }

  @Override
  public void initDefaultCommand() {

  }
}

