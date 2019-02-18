/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Utilities;

public class SetIntakeJoystick extends Command {

  /**
   * Runs the intake at the desired speed for the desired amount of time
   *
   * @param speed the speed to run the intake at as a fraction of its max speed
   * @param time  the time to run the intake for in seconds
   */
  public SetIntakeJoystick() {
    requires(Robot.m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    Robot.m_intake.runAll(Utilities.deadzone
      ((Robot.m_oi.getDriveStick().getRawAxis(2) - Robot.m_oi.getDriveStick().getRawAxis(3))*0.6));
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
