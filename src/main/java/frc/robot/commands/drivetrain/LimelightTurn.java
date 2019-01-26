/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;

//TODO : Tuning

/**
 * Turns the robot so that it is aligned with whatever the robot is detecting on the limelight
 * @author Jacob4649
 */
public class LimelightTurn extends PIDCommand {

  private double m_maxSpeed;

  /**
   * Turns the robot so that it is aligned with whatever the robot is detecting on
   * the limelight
   *
   * @param maxSpeed the maximum speed of the turn as a fraction
   */
  public LimelightTurn(double maxSpeed) {
    super(RobotConstants.LIMELIGHT_TURN_KP, RobotConstants.LIMELIGHT_TURN_KI, RobotConstants.LIMELIGHT_TURN_KD);
    requires(Robot.m_driveTrain);

    getPIDController().setInputRange(-20.5, 20.5);
    getPIDController().setOutputRange(0.2, 0.8); //Dummy numbers, will need to be updates
    getPIDController().setContinuous(false);
    getPIDController().setAbsoluteTolerance(4.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    getPIDController().enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return getPIDController().onTarget();
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

  // returns the value the pid controller is using as an input
  @Override
  protected double returnPIDInput() {
    return Robot.m_limelight.getOffset()[0]; // returns the horizontal distance from center of the object detected by the limelight
  }

  // processes the pid output, sends new values to motors and stuff
  @Override
  protected void usePIDOutput(double output) {
    // drives at the outputted speed, or the max speed
    Robot.m_driveTrain.mecanumDrive(0.0, 0.0, output);
  }
}