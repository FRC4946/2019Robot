/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.drivetrain.SetDriveTrain;

public class MidStageClimb extends Command {
  Timer m_climbTimer = new Timer();

  SetDriveTrain m_driveForwards = new SetDriveTrain(0.15);
  public MidStageClimb() {
    requires(Robot.m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_climbTimer.stop();
    m_climbTimer.reset();
    m_climbTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_climbTimer.get() <= 1) {
      if (Robot.m_climber.getFrontClimberHeight() < 2.83)
        Robot.m_climber.setFront(0.6);
      else
        Robot.m_climber.setFront(0.0);
    } else if (m_climbTimer.get() <= 2.5) {
      Robot.m_climber.setFront(-0.6);
      if (Robot.m_climber.getBackClimberHeight() < 2.6)
        Robot.m_climber.setBack(0.6);
      else
        Robot.m_climber.setBack(0.0); 
    } else {
      Robot.m_climber.setFront(0.0);
      Robot.m_climber.setBack(-0.6);
    }
    if (!m_driveForwards.isRunning())
      m_driveForwards.start();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_driveForwards.cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
