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

public class SetElevator extends Command {

  private double m_speed;

  public SetElevator(double speed) {
    requires(Robot.m_elevator);
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.m_elevator.disablePID(); (needed for when PID is added)
		Robot.m_elevator.setBrake(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_MINIMUM_HEIGHT && m_speed < 0
      || Robot.m_elevator.getHeight() > RobotConstants.ELEVATOR_MAXIMUM_HEIGHT && m_speed > 0) {
			Robot.m_elevator.setElevator(0);
    } else {
      Robot.m_elevator.setElevator(m_speed);
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
    Robot.m_elevator.setElevator(0);
  }
  
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
