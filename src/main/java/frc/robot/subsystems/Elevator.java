/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  private CANSparkMax m_motor; 
  private Solenoid m_brake;
  private AnalogPotentiometer m_analogPot;

	private boolean m_isBrake = false;


public Elevator (){

  m_motor = new CANSparkMax(RobotMap.CAN_RUN_ELEVATOR, MotorType.kBrushless);
  m_brake = new Solenoid(RobotMap.PCM_ELEVATOR_BREAK);
  m_analogPot = new AnalogPotentiometer(RobotMap.ANALOG_ELEVATOR_POT, 
  RobotConstants.ELEVATOR_SCALING_VALUE, RobotConstants.ELEVATOR_OFFSET_VALUE);

}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public double getHeight() {
		return m_analogPot.get();
	}

	public void setBrake(boolean isBrake) {
		m_isBrake = isBrake;
		m_brake.set(!m_isBrake);
		// if (m_isBrake)
		// m_brake.set(Value.kForward);
		// else
		// m_brake.set(Value.kReverse);
	}

	/**
	 * Manually sets the speed of the motors.
	 * 
	 * @param speed
	 *            The fraction of the motor's maximum speed the motors are to spin
	 *            at. Ranges between -1.0 and 1.0
	 */
	public void set(double speed) {

		m_motor.set(speed);

	}

	/**
	 * @return the speed of the elevator motors
	 */
	public double getSpeed() {
		return m_motor.get();
	}
}
