/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends Command {
  public Vision() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
    //requires(Robot.LimelightObj);
    
  }
  double KpDistance;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double KpDistance = -0.1;  // Proportional control constant for distance
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Periodic Updates To Info
    Robot.LimelightObj.xOffset = Robot.LimelightObj.tx.getDouble(0.0); // Coordinate updates
    Robot.LimelightObj.yOffset = Robot.LimelightObj.ty.getDouble(0.0);
    Robot.LimelightObj.area = Robot.LimelightObj.ta.getDouble(0.0);
    Robot.LimelightObj.detected = Robot.LimelightObj.tv.getDouble(0); // see if limelight has detected anytihng
    SmartDashboard.putNumber("LimelightX", Robot.LimelightObj.xOffset); // Dashboard updates
    SmartDashboard.putNumber("LimelightY", Robot.LimelightObj.yOffset);
    SmartDashboard.putNumber("LimelightArea", Robot.LimelightObj.area);

    if(Robot.m_oi.getdriveStick().getRawButton(/*Button Number*/1) == true) {
      // Moving To Target
      double driveAdj = KpDistance*(-1*Robot.LimelightObj.findDistance());
      /*Drive Commands*/ 
    }
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
}
