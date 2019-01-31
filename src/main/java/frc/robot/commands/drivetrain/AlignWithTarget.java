/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.DummyPIDOutput;
import frc.robot.Robot;

//TODO : Tuning

public class AlignWithTarget extends PIDCommand {
  
  PIDController gyroController;
  DummyPIDOutput dummyOutput;

  public AlignWithTarget() {

    super(0.00045, 0.006, 0.0);
    requires(Robot.m_driveTrain);

    dummyOutput = new DummyPIDOutput();

    gyroController = new PIDController(0.005, 0.0, 0.0, Robot.m_driveTrain.getGyro(), dummyOutput);
    gyroController.setInputRange(0, 360.0);
    gyroController.setContinuous(true);
    gyroController.setOutputRange(-0.2, 0.2);
    gyroController.setSetpoint(Robot.m_driveTrain.getGyroAngleAbs());
    gyroController.setAbsoluteTolerance(4);

    getPIDController().setInputRange(-20.5, 20.5);
    getPIDController().setOutputRange(-0.1, 0.1); 
    getPIDController().setContinuous(false);
    getPIDController().setAbsoluteTolerance(1.0);
    getPIDController().setSetpoint(0.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    gyroController.enable();
    Robot.m_limelight.setLED(true);
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