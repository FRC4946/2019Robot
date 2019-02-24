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
import frc.robot.commands.intakeelbow.SetIntakePos;

public class SetElevator extends Command {

  private double m_speed;

  public SetElevator(double speed) {
    requires(Robot.m_elevator);
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (((Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_CONFLICT_HEIGHT && m_speed > 0) || (Robot.m_elevator.getHeight() > RobotConstants.ELEVATOR_CONFLICT_HEIGHT && m_speed < 0)) && !(Math.abs(Robot.m_intakeElbow.getPos() - RobotConstants.INTAKE_POT_DOWN) < 0.2)) { //elevator is moving towards the conflict zone and intake is not down
      new SetIntakePos(RobotConstants.INTAKE_POT_DOWN, 0.5).start();
      Robot.m_elevator.setElevator(m_speed*0.2);
    } else {
      Robot.m_elevator.setElevator(m_speed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_MINIMUM_HEIGHT && m_speed < 0
    || Robot.m_elevator.getHeight() > RobotConstants.ELEVATOR_MAXIMUM_HEIGHT && m_speed > 0;
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
