/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class DoubleClimb extends Command {
  SetIntakePos m_setIntakeUp = new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.2);
  public DoubleClimb() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_climber.getFrontClimberHeight() < RobotConstants.FRONT_CLIMBER_MID_LEVEL_HEIGHT)
      Robot.m_climber.setFront(Math.abs(Robot.m_climber.getFrontClimberHeight() - RobotConstants.FRONT_CLIMBER_DOUBLE_CLIMB_HEIGHT) > 0.2 ? 0.5 : 0.1);
    else
      Robot.m_climber.setFront(0.0);
      
    if (Robot.m_climber.getBackClimberHeight() < RobotConstants.BACK_CLIMBER_MID_LEVEL_HEIGHT)
      Robot.m_climber.setBack(Math.abs(Robot.m_climber.getBackClimberHeight() - RobotConstants.BACK_CLIMBER_DOUBLE_CLIMB_HEIGHT) > 0.2 ? 0.4 : 0.1);
    else
      Robot.m_climber.setBack(0.0);
    if (!m_setIntakeUp.isRunning() && Math.abs(Robot.m_intakeElbow.getPos() - RobotConstants.INTAKE_POT_UP) > 50)
      m_setIntakeUp.start();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stopClimber();
    m_setIntakeUp.cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
