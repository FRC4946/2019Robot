/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * TODO: Document
 */
public class GrabberSubsystem extends Subsystem {

  private Solenoid m_grabSolenoid;
  private boolean m_grabberUp;

  public GrabberSubsystem() {
    m_grabSolenoid = new Solenoid(RobotMap.PCM_SOLGRABBER);
  }

  public void setGrabber(boolean isUP) {
    m_grabberUp = isUP;
    m_grabSolenoid.set(m_grabberUp);
  }

  public boolean getGrabberPosition() {
    return m_grabberUp;
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new MySpecialCommand());
  }
}
