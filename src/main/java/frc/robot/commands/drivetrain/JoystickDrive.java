/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.DummyPIDOutput;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.Utilities;

public class JoystickDrive extends Command {

  PIDController m_gyroController; 
  boolean isSidewaysStrafing;

  public JoystickDrive() {
    requires(Robot.m_driveTrain);
    isSidewaysStrafing = false;
    m_gyroController = new PIDController(0.0125, 
      0.0, 0.00001, 
      Robot.m_driveTrain.getGyro(), new DummyPIDOutput());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_gyroController.setInputRange(0, 360);
    m_gyroController.setOutputRange(-0.25, 0.25);
    m_gyroController.setContinuous(true);
    m_gyroController.setAbsoluteTolerance(1);
    m_gyroController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
/*
    if(Robot.m_oi.getDriveStick().getPOV() == 0) {

      isSidewaysStrafing = false;
      Robot.m_driveTrain.mecanumDrive(0.6, 0, 0);

    } else if (Robot.m_oi.getDriveStick().getPOV() == 90) {

      if(!isSidewaysStrafing) {
        m_gyroController.setSetpoint(Robot.m_driveTrain.getGyroAngle());
        isSidewaysStrafing = true;
      }
      Robot.m_driveTrain.mecanumDrive(0, 0.6, m_gyroController.get() - 0.01);

    } else if (Robot.m_oi.getDriveStick().getPOV() == 180) {

      isSidewaysStrafing = false;
      Robot.m_driveTrain.mecanumDrive(-0.6, 0, 0);

    } else if (Robot.m_oi.getDriveStick().getPOV() == 270) {

      if(!isSidewaysStrafing) {
        m_gyroController.setSetpoint(Robot.m_driveTrain.getGyroAngle());
        isSidewaysStrafing = true;
      }
      Robot.m_driveTrain.mecanumDrive(0, -0.6, m_gyroController.get()*0.8 - 0.01);

      
    } else {
      */
      isSidewaysStrafing = false;
      Robot.m_driveTrain.mecanumDrive(
        Utilities.deadzone(-Robot.m_oi.getDriveStick().getRawAxis(1), 
          Math.abs(0.2*Robot.m_oi.getDriveStick().getRawAxis(0)) + RobotConstants.DEFAULT_DEADZONE),
        Utilities.deadzone(Robot.m_oi.getDriveStick().getRawAxis(0), 
          Math.abs(0.2*Robot.m_oi.getDriveStick().getRawAxis(1)) + RobotConstants.DEFAULT_DEADZONE),
        Utilities.deadzone(Robot.m_oi.getDriveStick().getRawAxis(4),
          RobotConstants.DEFAULT_DEADZONE));
    //}
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
