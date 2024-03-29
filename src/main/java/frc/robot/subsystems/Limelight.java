/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.limelight.SetLimelightLED;

/**
 * Limelight Object
 * 
 * Pipelines:
 * 0 - Driving
 * 1 - Pickup Hatch
 * 2 - Drop Hatch
 *
 * @author Jacob4649
 */
public class Limelight extends Subsystem{

  public NetworkTable m_networkTable;
  public NetworkTableEntry m_tx;
  public NetworkTableEntry m_ty;
  public NetworkTableEntry m_ta;
  public NetworkTableEntry m_tv;
  public NetworkTableEntry m_ts;
  public double m_detected;
  public double m_xOffset;
  public double m_yOffset;
  public double m_area;

  /**
   * Initializes all network table entries, and creates object Turns on vision and
   * LEDs by default
   */
  public Limelight() {
    m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    setVision(true);

    m_tx = m_networkTable.getEntry("tx");
    m_ty = m_networkTable.getEntry("ty");
    m_ta = m_networkTable.getEntry("ta");
    m_tv = m_networkTable.getEntry("tv");
    m_ts = m_networkTable.getEntry("ts");
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new SetLimelightLED(false));
  }

  /**
   * Returns whether or not the limelight has any valid targets
   *
   * @return true if the limelight detects a valid target
   */
  public boolean getHasTarget() {
    return (m_tv.getDouble(0) == 1);
  }

  /**
   * Returs the coordinates of the centre of the targeting box
   *
   * @return An array containing the x offset and y offset of the target {xOffset,
   *         yOffset}
   */
  public double[] getOffset() {
    return new double[] { m_tx.getDouble(0), m_ty.getDouble(0) };
  }

  /**
   * Returns the area of the target as a percentage of the whole image (0% to
   * 100%)
   *
   * @return  the area of the target
   */
  public double getTargetArea() {
    return m_ta.getDouble(0);
  }

  /**
   * Returns the skew of the targeting box from -90 to 0 degrees
   *
   * @return the skew of the targeting box
   */
  public double getTargetSkew() {
    return m_ts.getDouble(0);
  }

  /**
   * Returns true if the LED is on and false if it isn't 
   * @return true if the LED is on
   */
  public boolean getLED() {
    return (!(m_networkTable.getEntry("ledMode").getDouble(0.0) == 1.0));
  }

  /**
   * Turns the leds on the limelight on or off
   *
   * @param on Should the LEDs on the limelight be on
   */
  public void setLED(boolean on) {
    if (on) {
      m_networkTable.getEntry("ledMode").setNumber(3);
    } else {
      m_networkTable.getEntry("ledMode").setNumber(1);
    }

  }

  /**
   * Sets the leds on the limelight to the pipeline default
   */
  public void setLEDDefault() {
    m_networkTable.getEntry("ledMode").setNumber(0);
  }

  /**
   * Enables or disables vision processcing on the limelight
   *
   * @param on Should vision processing be on or off
   */
  public void setVision(boolean on) {
    if (on) {
      m_networkTable.getEntry("camMode").setNumber(0);
    } else {
      m_networkTable.getEntry("camMode").setNumber(1);
    }

  }

  /**
   * Sets the limelight to the desired vision processcing pipeline
   *
   * @param pipeline the pipeline to set the limelight to from 0 to 9
   * @throws IndexOutOfBoundsException if the pipeine number is not between 0 and 9
   */
  public void setPipeline(int pipeline) {

    if (pipeline > 9 || pipeline < 0) {
      throw new IndexOutOfBoundsException("Pipeline Index Is Out Of Bounds");
    }
    m_networkTable.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Estimates the distance to a detected strip of reflective tape on the field
   * floor
   *
   * @return estimated distance in metres
   */
  public double findDistance() {

    // having a findDistance() function or an analogous function may be useful,
    // though we will likely
    // not be using these calculations

    // it may be more useful to just use tx, ty, and ta as raw values?? calculations
    // may be inaccurate

    double height = 1; // In metres
    double angle = 45; // angle of mounting respective of roof

    return (height * -1) / Math.tan(angle + m_ty.getDouble(0));
  }
}
