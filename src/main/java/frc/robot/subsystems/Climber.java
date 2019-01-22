/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Talon m_leftSide;
  private Talon m_rightSide;

  public Climber(){

    m_leftSide = new Talon(RobotMap.CAN_LIFT_LEFT_SIDE);
    m_rightSide = new Talon(RobotMap.CAN_LIFT_RIGHT_SIDE);
         
  }

  public void setClimberSpeed (double climberSpeed){

    m_leftSide.set(climberSpeed);
    m_rightSide.set(climberSpeed);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
