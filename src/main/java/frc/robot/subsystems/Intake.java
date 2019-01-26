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

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {

  private CANSparkMax m_outerLeft = new CANSparkMax(RobotMap.CAN_INTAKE_OUTER_LEFT, MotorType.kBrushless);
  private CANSparkMax m_outerRight = new CANSparkMax(RobotMap.CAN_INTAKE_OUTER_RIGHT, MotorType.kBrushless);
  private CANSparkMax m_innerLeft = new CANSparkMax(RobotMap.CAN_INTAKE_INNER_LEFT, MotorType.kBrushless);
  private CANSparkMax m_innerRight = new CANSparkMax(RobotMap.CAN_INTAKE_INNER_RIGHT, MotorType.kBrushless);

  private DigitalInput m_bannerSensor = new DigitalInput(RobotMap.INTAKE_BANNER_SENSOR);

  /**
   * Runs the outer intake at the desired speed
   *
   * @param speed the speed to run the outer intake at as a fraction of its max
   *              speed negative is inwards positive is outwards
   */
  public void runOuter(double speed) {
    m_outerLeft.set(speed);
    m_outerRight.set(speed);
  }

  /**
   * Runs the inner intake at the desired speed
   *
   * @param speed the speed to run the inner intake at as a fraction of its max
   *              speed negative is inwards positive is outwards
   */
  public void runInner(double speed) {
    m_innerLeft.set(speed);
    m_innerRight.set(speed);
  }

  /**
   * Runs both intakes at the desired speed
   *
   * @param speed the speed to run both intakes at as a fraction of their top
   *              speed negative is inwards positive is outwards
   */
  public void runAll(double speed) {
    runOuter(speed);
    runInner(speed);
  }

  /**
   * Stops the outer intake
   */
  public void stopOuter() {
    runOuter(0.0);
  }

  /**
   * Stops the inner intake
   */
  public void stopInner() {
    runInner(0.0);
  }

  /**
   * Stops both the outer and the inner intake
   */
  public void stopAll() {
    stopInner();
    stopOuter();
  }

  /**
   * Returns the state of the banner sensor on the intake
   *
   * @return false if no ball is detected, true if a ball is detected
   */
  public boolean getIsBall() {
    return !m_bannerSensor.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
