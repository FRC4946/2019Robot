/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabberarm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class SetArmSpeed extends Command {

  double m_speed;

  public SetArmSpeed(double speed) {
    requires(Robot.m_grabberArm);
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_grabberArm.setArm(m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return (Robot.m_grabberArm.getPos() >= RobotConstants.GRABBER_ARM_IN && m_speed < 0)
      //|| (Robot.m_grabberArm.getPos() <= RobotConstants.GRABBER_ARM_OUT && m_speed > 0);
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_grabberArm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
