/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;

//TODO : Tuning

/**
 * Turns the robot so that it is aligned with whatever the robot is detecting on
 * the limelight
 *
 * @author Jacob4649
 */
public class RotateToTarget extends PIDCommand {

  double m_maxSpeed = 0.2;
  double m_maxSpeedStrafe = 0.3;

  /**
   * Turns the robot so that it is aligned with whatever the robot is detecting on
   * the limelight
   *
   */
  public RotateToTarget() {
    super(RobotConstants.PID_ROTATE_TO_TARGET_P, 
      RobotConstants.PID_ROTATE_TO_TARGET_I, RobotConstants.PID_ROTATE_TO_TARGET_D);
    
    requires(Robot.m_driveTrain);
    requires(Robot.m_limelight);

    //getPIDController().setInputRange(-59.6, 59.6);
    getPIDController().setOutputRange(-m_maxSpeed, m_maxSpeed); // Dummy numbers, will need to be updates
    getPIDController().setContinuous(false);
    getPIDController().setAbsoluteTolerance(4.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    if (Robot.m_grabber.getGrabberOut()) { //has hatch
      Robot.m_limelight.setPipeline(RobotConstants.LIMELIGHT_PLACE_PIPELINE); //switches to placement pipeline
    } else {
      Robot.m_limelight.setPipeline(RobotConstants.LIMELIGHT_GRAB_PIPELINE); //switches to pickup pipeline
    }

    Robot.m_limelight.setLED(true);
    getPIDController().enable();
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
    Robot.m_limelight.setPipeline(RobotConstants.LIMELIGHT_DRIVE_PIPELINE); //reactivates driving pipeline (high exposure)
    Robot.m_limelight.setLED(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  // Returns the value the pid controller is using as an input, in this case
  // the horizontal distance from center of the object detected by the limelight
  @Override
  protected double returnPIDInput() {
    return Robot.m_limelight.getOffset()[0];
    //does strafing rely on dual target?? gah idk what to put in getOffset[] for that
  }

  // processes the pid output, sends new values to motors and stuff
  @Override
  protected void usePIDOutput(double output) {
    // drives at the outputted speed, or the max speed

    //if(Math.abs(output) < someConst) { 
        //change PID Values for strafing?? use boolean flag so that they only change once
        //Robot.m_driveTrain.mecanumDrive(0.0, -output, 0.0);
    //} else {
        //Robot.m_driveTrain.mecanumDrive(0.1, 0.0, -output);
    //}

    if (Robot.m_limelight.getTargetArea() > RobotConstants.LIMELIGHT_STRAFE_THRESHOLD && Math.abs(returnPIDInput()) > RobotConstants.LIMELIGHT_ERROR_THRESHOLD) {
      getPIDController().setPID(RobotConstants.PID_STRAFE_TO_TARGET_P, 
      RobotConstants.PID_STRAFE_TO_TARGET_I, RobotConstants.PID_STRAFE_TO_TARGET_D);
      getPIDController().setOutputRange(-m_maxSpeedStrafe, m_maxSpeedStrafe);
      Robot.m_driveTrain.mecanumDrive(0.1, -output, 0.0);
    } else {
      getPIDController().setPID(RobotConstants.PID_ROTATE_TO_TARGET_P, 
      RobotConstants.PID_ROTATE_TO_TARGET_I, RobotConstants.PID_ROTATE_TO_TARGET_D);
      getPIDController().setOutputRange(-m_maxSpeed, m_maxSpeed);
      Robot.m_driveTrain.mecanumDrive(0.1, 0.0, -output);
    }
  }
}
