/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.RobotConstants;

public class JoystickDriveAbs extends Command {

  public JoystickDriveAbs() {
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if(Robot.m_oi.getDriveStick().getPOV() == 0) {
      Robot.m_driveTrain.mecanumDriveAbs(0.25, 0, 0);
    } else if (Robot.m_oi.getDriveStick().getPOV() == 90) {
      Robot.m_driveTrain.mecanumDriveAbs(0, 0.25, 0);
    } else if (Robot.m_oi.getDriveStick().getPOV() == 180) {
      Robot.m_driveTrain.mecanumDriveAbs(-0.25, 0, 0);
    } else if (Robot.m_oi.getDriveStick().getPOV() == 270) {
      Robot.m_driveTrain.mecanumDriveAbs(0, -0.25, 0);
    } else {

      Robot.m_driveTrain.mecanumDriveAbs(
        Utilities.deadzone(-Math.pow(Robot.m_oi.getDriveStick().getRawAxis(1), 2), 
          Math.abs(0.2*Robot.m_oi.getDriveStick().getRawAxis(0)) + RobotConstants.DEFAULT_DEADZONE),
        Utilities.deadzone(Math.pow(Robot.m_oi.getDriveStick().getRawAxis(0), 2),
          Math.abs(0.2*Robot.m_oi.getDriveStick().getRawAxis(1)) + RobotConstants.DEFAULT_DEADZONE),
        Utilities.deadzone(Robot.m_oi.getDriveStick().getRawAxis(4), 
          RobotConstants.DEFAULT_DEADZONE));
          
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
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_driveTrain.stop();
  }
}
