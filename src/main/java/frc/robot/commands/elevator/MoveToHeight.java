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
  SetArmToPos m_setArmOut = new SetArmToPos(RobotConstants.GRABBER_ARM_OUT, 0.8);

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

    if(m_height >= RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW && Robot.m_elevator.getHeight() <= RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW
      && m_speedInit < 0) {
      Robot.m_elevator.setElevator(0);
    } else {
      Robot.m_elevator.setElevator(m_speedInit);
    }

    if (Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_OUT) > 0.3) {
      m_setArmOut.start();
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