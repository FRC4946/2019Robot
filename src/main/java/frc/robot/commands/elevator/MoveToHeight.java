/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class MoveToHeight extends PIDCommand {
  
  double m_height, m_maxSpeed;

  public MoveToHeight(double height, double maxSpeed) {

    super(0.2, 0.0, 0.0);
    requires (Robot.m_elevator);
    m_height = height;
    m_maxSpeed = maxSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    getPIDController().setInputRange(RobotConstants.ELEVATOR_MINIMUM_HEIGHT, RobotConstants.ELEVATOR_MAXIMUM_HEIGHT);
    getPIDController().setOutputRange(-m_maxSpeed, m_maxSpeed);
    getPIDController().setAbsoluteTolerance(1.0);
    getPIDController().setSetpoint(m_height);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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

  @Override 
  protected double returnPIDInput() {
    return Robot.m_elevator.getHeight();
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.m_elevator.setElevator(output);
  }
}
