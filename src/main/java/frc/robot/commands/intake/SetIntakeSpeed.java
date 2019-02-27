/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.grabberarm.SetArmToPos;

public class SetIntakeSpeed extends Command {
  double m_speed;
  public SetIntakeSpeed(double speed) {
    requires(Robot.m_intake);

    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intake.runAll(m_speed);

    if(Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT
      && m_speed < 0 && Robot.m_grabberArm.getPos() < RobotConstants.GRABBER_ARM_HOLD_BALL) {

        new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_BALL, 0.8);
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
    Robot.m_intake.stopAll();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
