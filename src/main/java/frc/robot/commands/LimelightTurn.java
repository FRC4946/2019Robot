/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;

//TODO : Add handling for cases where limelight is detecting more than one object

public class LimelightTurn extends PIDCommand {

  double maxSpeed;

  /** Turns the robot so that it is aligned with whatever the robot is detecting on the limelight
   *  
   * @param maxSpeed the maximum speed of the turn as a fraction
   */
  public LimelightTurn(double maxSpeed) {
    super(RobotConstants.LIMELIGHT_TURN_KP, RobotConstants.LIMELIGHT_TURN_KI, RobotConstants.LIMELIGHT_TURN_KD);
    requires(Robot.m_driveTrain);

    this.maxSpeed = maxSpeed; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
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
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  //returns the value the pid controller is using as an input
  @Override
  protected double returnPIDInput() {
    return 0; //returns the distance from center of the object detected by the limelight 
  }

  //processes the pid output, sends new values to motors and stuff
  @Override
  protected void usePIDOutput(double output) {
    Robot.m_driveTrain.mecanumDrive(0.0, 0.0, Math.min(output, maxSpeed)); //drives at the outputted speed, or the max speed
  }
}
