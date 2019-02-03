/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.deprecated;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class PIDDriveStraightDist extends PIDCommand {

  double m_dist, m_maxSpeed;
  boolean m_isHorizontal; 
  PIDController m_frontLeft, m_backLeft, m_frontRight, m_backRight;

  public PIDDriveStraightDist(double dist, double maxSpeed, boolean isHorizontal) {
    
    super(0.02, 0, 0);

    requires(Robot.m_driveTrain);

    m_dist = dist;
    m_maxSpeed = maxSpeed;
    m_isHorizontal = isHorizontal;

    /*
    m_frontLeft = new PIDController(RobotConstants.CAN_DRIVE_LEFT_FRONT_KP, RobotConstants.CAN_DRIVE_LEFT_FRONT_KI, 
      RobotConstants.CAN_DRIVE_LEFT_FRONT_KD, Robot.m_driveTrain.getLeftFrontEnc(), Robot.m_driveTrain.getLeftFront());
    
    m_frontRight = new PIDController(RobotConstants.CAN_DRIVE_RIGHT_FRONT_KP, RobotConstants.CAN_DRIVE_RIGHT_FRONT_KI,
      RobotConstants.CAN_DRIVE_RIGHT_FRONT_KD, Robot.m_driveTrain.getRightFrontEnc(), Robot.m_driveTrain.getRightFront());
    
    m_backLeft = new PIDController(RobotConstants.CAN_DRIVE_LEFT_BACK_KP, RobotConstants.CAN_DRIVE_LEFT_BACK_KI, 
      RobotConstants.CAN_DRIVE_LEFT_BACK_KD, Robot.m_driveTrain.getLeftFrontEnc(), Robot.m_driveTrain.getLeftFront());

    m_backRight = new PIDController(RobotConstants.CAN_DRIVE_RIGHT_BACK_KP, RobotConstants.CAN_DRIVE_RIGHT_BACK_KI,
      RobotConstants.CAN_DRIVE_RIGHT_BACK_KD, Robot.m_driveTrain.getRightBackEnc(), Robot.m_driveTrain.getRightBack());
    */
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveTrain.resetEncs();
    setSetpoint(m_dist);
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
    if(m_isHorizontal) {
      return - Robot.m_driveTrain.getLeftBackEnc().getDistance() + Robot.m_driveTrain.getLeftFrontEnc().getDistance() 
        - Robot.m_driveTrain.getRightFrontEnc().getDistance() + Robot.m_driveTrain.getRightBackEnc().getDistance();
    } else {
      return Robot.m_driveTrain.getLeftBackEnc().getDistance() + Robot.m_driveTrain.getLeftFrontEnc().getDistance() 
        + Robot.m_driveTrain.getRightFrontEnc().getDistance() + Robot.m_driveTrain.getRightBackEnc().getDistance();
    }
  }

  @Override
  protected void usePIDOutput(double output) {

    output = Math.abs(output) > Math.abs(m_maxSpeed) ? m_maxSpeed : output;

    if(m_isHorizontal) {
      Robot.m_driveTrain.getLeftFront().pidWrite(output);
      Robot.m_driveTrain.getRightFront().pidWrite(-output);
      Robot.m_driveTrain.getLeftBack().pidWrite(-output);
      Robot.m_driveTrain.getRightBack().pidWrite(output);
    } else {
      Robot.m_driveTrain.getLeftFront().pidWrite(output);
      Robot.m_driveTrain.getRightFront().pidWrite(output);
      Robot.m_driveTrain.getLeftBack().pidWrite(output);
      Robot.m_driveTrain.getRightBack().pidWrite(output);
    }
  }
}
