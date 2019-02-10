/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeelbow;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ToggleOuterIntakeElbow extends Command {

  double m_speed;

  public ToggleOuterIntakeElbow(double speed) {
    requires(Robot.m_intake);
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(Robot.m_intakeElbow.getElbowIsUp()) {
      Robot.m_intakeElbow.setElbow(-m_speed);
    } else {
      Robot.m_intakeElbow.setElbow(m_speed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
  
    if(Robot.m_intakeElbow.getElbowIsUp()) { 
      return Robot.m_intakeElbow.getPot() >= RobotConstants.DOWNWARDS_OUTER_INTAKE_POSITION;
    } else {
      return Robot.m_intakeElbow.getPot() <= RobotConstants.UPWARDS_OUTER_INTAKE_POSITION;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intakeElbow.setElbow(0.0);
    Robot.m_intakeElbow.setElbowIsUp(!Robot.m_intakeElbow.getElbowIsUp());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_intakeElbow.setElbow(0.0);
  }
}
