/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AbsTurn extends Command {

  double angle; //angle to turn to in degrees
  boolean m_turnLeft; //whether to turn left or not


  /** Turns the robot on the spot to the gyro angle provided
   * 
   * @param turn the angle to turn to in degrees
   */
  public AbsTurn(double angle) {
    requires(Robot.m_driveTrain);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_turnLeft = ((angle - Robot.m_driveTrain.getGyroAngle()) > 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_turnLeft) {
      if (angle - Robot.m_driveTrain.getGyroAngle() >= 180) {
       Robot.m_driveTrain.mecanumDrive(0.0, 0.0, 0.3);
      } 
      else {
        Robot.m_driveTrain.mecanumDrive(0.0, 0.0, -0.3);
      }
    } 
    else {
     if (Robot.m_driveTrain.getGyroAngle() >= 180){
       Robot.m_driveTrain.mecanumDrive(0.0, 0.0, -0.3);
     }
     else {
       Robot.m_driveTrain.mecanumDrive(0.0, 0.0, 0.3);
     }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.m_driveTrain.getGyroAngle() - angle) < 2); //within 2 degrees
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
    end();
  }
}
