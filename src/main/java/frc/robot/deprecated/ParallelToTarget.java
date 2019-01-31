/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.deprecated;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

public class ParallelToTarget extends PIDCommand {
  public ParallelToTarget() {
    super(0.2, 0.0, 0.0);
    requires(Robot.m_driveTrain);

    getPIDController().setInputRange(-90.0, 0);
    getPIDController().setOutputRange(-0.3, 0.3); 
    getPIDController().setContinuous(false);
    getPIDController().setAbsoluteTolerance(4.0);
    getPIDController().setSetpoint(0.0);
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

  @Override
  protected double returnPIDInput() {
    return Robot.m_limelight.getTargetSkew();
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.m_driveTrain.mecanumDrive(0.0, 0.0, -output);
  }
}
