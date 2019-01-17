/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.driveTrain;
import frc.robot.commands.Vision;



/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public NetworkTable table; // Initialising In Global Scope
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry ta;
  public NetworkTableEntry tv;
  public double detected;
  public double xOffset;
  public double yOffset;
  public double area;


  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight"); // Initialising In Global Scope
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    double detected;
    double xOffset;
    double yOffset;
    double area;
    double KpDistance = -0.1;  // Proportional control constant for distance
  }

  public double findDistance() {
    double height = 1; // In metres
    double angle = 45; // angle of mounting respective of roof
    double distance; // distance from object to robot

    distance = (height*-1) / Math.tan(angle+yOffset);

    return(distance);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Vision());
  }
}
