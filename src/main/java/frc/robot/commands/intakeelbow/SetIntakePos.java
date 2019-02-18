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

public class SetIntakePos extends Command {

  double m_desiredPos, m_speed;

  public SetIntakePos(double desiredPos, double speed) {
    requires(Robot.m_intakeElbow);
    m_desiredPos = desiredPos;
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_desiredPos = Math.min(m_desiredPos, RobotConstants.INTAKE_POT_UP);
    m_desiredPos = Math.max(m_desiredPos, RobotConstants.INTAKE_POT_DOWN);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intakeElbow.setElbow
      (Math.signum(Robot.m_intakeElbow.getPos() - m_desiredPos)*m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.m_intakeElbow.getPos() - m_desiredPos) < 2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intakeElbow.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
