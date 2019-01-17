/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class GrabberSubsystem extends Subsystem {

  private Solenoid m_grabSolenoid;
  private boolean m_grabberUp;

  public GrabberSubsystem(){

    m_grabSolenoid = new Solenoid (RobotMap.PCM_SOLGRABBER);

  }
  

  public void setGrabber(boolean isUP) {
		
		if (isUP) {
			m_grabSolenoid.set(true);
		}else {
			m_grabSolenoid.set(false);			
		}
			m_grabberUp = isUP;
	
  }
  public boolean getElbowPosition() {
		
		return m_grabberUp;
		}
	

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
