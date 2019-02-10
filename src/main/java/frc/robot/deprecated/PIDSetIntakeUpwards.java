/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.deprecated;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class PIDSetIntakeUpwards extends PIDCommand {

  double m_maxSpeed, m_place;
  boolean m_value;
  PIDController m_intakeAnalogPot;

  public PIDSetIntakeUpwards(double maxSpeed, double place, boolean value) {
   
    super (0.02, 0,0);
    requires(Robot.m_intakeElbow);

    m_maxSpeed = maxSpeed;
    m_place = place;
    m_value = value;

  }
  
 @Override
 protected void initialize() {
   setSetpoint(RobotConstants.UPWARDS_OUTER_INTAKE_POSITION);    
 }

  @Override
 protected void execute() {
 }

  @Override
 protected boolean isFinished() {
   return false;
 }

  @Override
 protected void end() {
 }


  @Override
  protected void interrupted() {
    
  }

  protected double returnPIDInput() {
   return Robot.m_intakeElbow.getPot();   
  }

  protected void usePIDOutput(double output){
   Robot.m_intakeElbow.setElbow(output);

  }

}
