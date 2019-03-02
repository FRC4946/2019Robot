/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.DummyPIDOutput;
import frc.robot.Robot;
import frc.robot.RobotConstants;

//TODO : Tuning

public class StrafeToTarget extends PIDCommand {
  
  PIDController gyroController;
  DummyPIDOutput dummyOutput;

  public StrafeToTarget() {

    super(RobotConstants.PID_STRAFE_TO_TARGET_P, 
      RobotConstants.PID_STRAFE_TO_TARGET_I, RobotConstants.PID_STRAFE_TO_TARGET_D);

    requires(Robot.m_driveTrain);
    
    gyroController = new PIDController(RobotConstants.PID_STRAFE_TO_TARGET_GYRO_P, 
      RobotConstants.PID_STRAFE_TO_TARGET_GYRO_I, RobotConstants.PID_STRAFE_TO_TARGET_GYRO_D, 
      Robot.m_driveTrain.getGyro(), new DummyPIDOutput()); 
    gyroController.setInputRange(0, 360.0);
    gyroController.setContinuous(true);
    gyroController.setOutputRange(-0.3, 0.3);
    gyroController.setSetpoint(Robot.m_driveTrain.getGyroAngle());
    gyroController.setAbsoluteTolerance(4);

    getPIDController().setInputRange(-59.6, 59.6);
    getPIDController().setOutputRange(-0.3, 0.3); 
    getPIDController().setContinuous(false);
    getPIDController().setAbsoluteTolerance(1.0);
    getPIDController().setSetpoint(0.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    gyroController.enable();
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
    gyroController.disable();
    getPIDController().disable();
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
    Robot.m_driveTrain.mecanumDrive(0.0, -output, 0.0);
  }
}
