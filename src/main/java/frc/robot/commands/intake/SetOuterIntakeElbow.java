/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class SetOuterIntakeElbow extends Command {
  
  boolean m_setToUp;
  double m_speed;

  public SetOuterIntakeElbow(boolean setToUp, double speed) {
    requires(Robot.m_intake);
    m_setToUp = setToUp;
    m_speed = Math.abs(speed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(m_setToUp) {
      Robot.m_intake.setElbow(m_speed);
    } else {
      Robot.m_intake.setElbow(-m_speed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_setToUp) {
      return Robot.m_intake.getPot() <= RobotConstants.UPWARDS_OUTER_INTAKE_POSITION;
    } else {
      return Robot.m_intake.getPot() >= RobotConstants.DOWNWARDS_OUTER_INTAKE_POSITION;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intake.setElbow(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
