/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // All for Vision
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends Command {
  public Vision() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
    
  }

  public double findDistance() {
    double height = 1; // In metres
    double angle = 45; // angle of mounting respective of roof
    double distance; // distance from object to robot

    distance = (height*-1) / Math.tan(angle+yOffset);

    return(distance);
  }

  NetworkTable table; // Initialising In Global Scope
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  double detected;
  double xOffset;
  double yOffset;
  double area;
  double KpDistance = -0.1;  // Proportional control constant for distance

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Periodic Updates To Info
    xOffset = tx.getDouble(0.0); // Coordinate updates
    yOffset = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    detected = tv.getDouble(0); // see if limelight has detected anytihng
    SmartDashboard.putNumber("LimelightX", xOffset); // Dashboard updates
    SmartDashboard.putNumber("LimelightY", yOffset);
    SmartDashboard.putNumber("LimelightArea", area);

    if(Robot.m_oi.getdriveStick().getRawButton(/*Button Number*/1) == true) {
      // Moving To Target
      double driveAdj = KpDistance*(-1*findDistance());
      /*Drive Commands*/ 
      Robot.m_driveTrain.mecanumDrive(driveAdj,0,0);
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
