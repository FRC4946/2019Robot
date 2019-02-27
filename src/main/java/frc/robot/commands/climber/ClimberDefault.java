/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ClimberDefault extends Command {
  public ClimberDefault() {
    requires(Robot.m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
    Robot.m_climber.getFrontPIDController().setOutputRange(-0.4, 0.4);
    Robot.m_climber.getBackPIDController().setOutputRange(-0.4, 0.4);
    Robot.m_climber.setFrontPIDController(0.2, 0.0, 0.0);
    Robot.m_climber.setBackPIDController(0.2, 0.0, 0.0);
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.m_oi.getOperatorStick().getPOV() == 90) {
      Robot.m_climber.setBack(0.5);
    } else if (Robot.m_oi.getOperatorStick().getPOV() == 270) {
      Robot.m_climber.setFront(-0.5);
    } else {
      
      /*
      if (Robot.m_climber.getFrontClimberHeight() < 7 && Robot.m_climber.getBackClimberHeight() < 7) {
        Robot.m_climber.getFrontPIDController().setReference(6, ControlType.kPosition);
        Robot.m_climber.getBackPIDController().setReference(6 + RobotConstants.CLIMBER_OFFSET, ControlType.kPosition);
      } else {
        Robot.m_climber.stopClimber();
      }*/
      Robot.m_climber.stopClimber();
    }
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
