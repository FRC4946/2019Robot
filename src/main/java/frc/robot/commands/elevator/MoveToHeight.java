/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.grabber.SetGrabber;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class MoveToHeight extends Command {
  
  double m_height, m_speed, m_speedInit;
  SetArmToPos m_setOut = new SetArmToPos(RobotConstants.GRABBER_ARM_OUT, 0.7);
  SetArmToPos m_setBall = new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_BALL, 0.7);
  SetIntakePos m_setUp = new SetIntakePos(RobotConstants.INTAKE_POT_BALL_HEIGHT, 0.25);

  public MoveToHeight(double height, double speed) {
    requires(Robot.m_elevator);

    m_height = height;
    m_speed = Math.min(Math.abs(speed), 0.9);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    m_speedInit = m_speed;
    m_speedInit *= Math.signum(m_height - Robot.m_elevator.getHeight());


    if(m_speedInit < 0) {
      m_speedInit *= 0.45;
    }
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 

    Robot.m_elevator.setElevator(m_speedInit);

    if (Robot.m_elevator.getHeight() > RobotConstants.ELEVATOR_THRESHOLD) {
      if (Math.abs(Robot.m_grabberArm.getPos()-RobotConstants.GRABBER_ARM_OUT) > 0.2) {
        m_setOut.start();
      }
    } else {
      if (Math.abs(Robot.m_grabberArm.getPos()-RobotConstants.GRABBER_ARM_HOLD_BALL) > 1.0) {
        m_setBall.start();
      } 
    }
    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (m_speedInit < 0) {
      return Robot.m_elevator.getHeight() <= (m_height + 0.02);
    } else {
      return Robot.m_elevator.getHeight() >= (m_height - 0.02);
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