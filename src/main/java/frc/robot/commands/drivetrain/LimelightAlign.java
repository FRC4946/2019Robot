/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

public class LimelightAlign extends PIDCommand {


  public LimelightAlign() {

    super(0.02, 0.0, 0.0);
    requires(Robot.m_driveTrain);
    this.getPIDController().setInputRange(-27.0, 27.0);
    this.getPIDController().setOutputRange(-0.2, 0.2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.setSetpoint(0.0);
    this.getPIDController().setSetpoint(0.0);
    this.getPIDController().enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(this.getPIDController().getError()) < 0.5;
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
  public double returnPIDInput() {
    return Robot.m_limelight.getOffset()[0];
  }

  @Override
  public void usePIDOutput(double output) {
    Robot.m_driveTrain.mecanumDrive(0.0, output, 0.0);
  }
}
