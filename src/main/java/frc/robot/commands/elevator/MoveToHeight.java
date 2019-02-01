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

public class MoveToHeight extends Command {
  
  //TODO: FINISH
  double m_height, m_speed;
	int onTargetCount = 0;

  public MoveToHeight(double height, double speed) {
    requires (Robot.m_elevator);
    m_height = height;
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double setpoint = m_height;

		// Limit the setpoint to the appropriate bounds, and apply it to the elevator
		setpoint = Math.min(setpoint, RobotConstants.ELEVATOR_MAXIMUM_HEIGHT);
		setpoint = Math.max(setpoint, RobotConstants.ELEVATOR_MINIMUM_HEIGHT);

    //Robot.m_elevator.setSetpoint(setpoint); (related to the PID part of the subsystem)

		// Limit the setpoint to the appropriate bounds, and apply it to the elevator
		m_height = Math.min(m_height, RobotConstants.ELEVATOR_MAXIMUM_HEIGHT);
    m_height = Math.max(m_height, RobotConstants.ELEVATOR_MINIMUM_HEIGHT);
    
    Robot.m_elevator.setElevator(m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
       // Robot.m_elevator.disablePID(); (needed for when PID is added)
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
