/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class MoveToHeight extends Command {
  
  boolean isBelowConflict;
  double m_height, m_speed, m_speedInit;

  public MoveToHeight(double height, double speed) {
    requires(Robot.m_elevator);

    m_height = height;
    m_speed = Math.min(Math.abs(speed), 0.9);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    isBelowConflict = Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT;
    m_speedInit = m_speed;
    m_speedInit *= Math.signum(m_height - Robot.m_elevator.getHeight());


    if(m_speedInit < 0) {
      m_speedInit *= 0.45;
    }

    if(m_height < RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW) {
      if(Math.abs(Robot.m_intakeElbow.getPos() - RobotConstants.INTAKE_POT_BALL_HEIGHT) > 10) {
        new SetIntakePos(RobotConstants.INTAKE_POT_BALL_HEIGHT, 0.25).start(); 
      } 
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 
    
    if(m_height >= RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW && Robot.m_elevator.getHeight() <= RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW) {
      Robot.m_elevator.setElevator(0);
    } else {
      Robot.m_elevator.setElevator(m_speedInit);
    }

    if(isBelowConflict != Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT) {
     
      if(Robot.m_elevator.getHeight() >= RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT && m_speedInit > 0) {

        if(Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_HATCH) > 0.2) {
          new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_HATCH, 0.8).start();
        }

        if(Math.abs(Robot.m_intakeElbow.getPos() - RobotConstants.INTAKE_POT_UP) > 30) {
          new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.3).start(); 
        } 

      } else if (Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT 
        && m_speedInit < 0 && m_height < RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW) {

        if(Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_BALL) > 0.2) {
          new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_BALL, 0.8).start();
        }
      }

      isBelowConflict = !isBelowConflict;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_speedInit < 0) {
      return Robot.m_elevator.getHeight() <= m_height + 0.02;
    } else {
      return Robot.m_elevator.getHeight() >= m_height - 0.02;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
