/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.deprecated;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * TODO: Document
 */
public class Limelight extends Subsystem {

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
  }

  public double findDistance() {
    double height = 1; // In metres
    double angle = 45; // angle of mounting respective of roof
    double distance; // distance from object to robot

    distance = (height * -1) / Math.tan(angle + ty.getDouble(0));

    return (distance);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Vision());
  }
}
