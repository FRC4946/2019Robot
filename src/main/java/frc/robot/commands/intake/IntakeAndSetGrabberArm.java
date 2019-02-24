/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.Utilities;
import frc.robot.commands.grabberarm.SetArmToPos;

public class IntakeAndSetGrabberArm extends CommandGroup {
  /**
   * Add your docs here.
   */

  double m_speed; 

  public IntakeAndSetGrabberArm() {
  }

  @Override
  protected void initialize() {
    if(Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_CONFLICT_HEIGHT && m_speed < 0) {
      addSequential(new SetArmToPos(RobotConstants.GRABBER_ARM_IN, 0.6), 0.5); 
    }
  }

  @Override
  protected void execute() {
    m_speed = Utilities.deadzone
    ((Robot.m_oi.getDriveStick().getRawAxis(2) - Robot.m_oi.getDriveStick().getRawAxis(3))*0.6);
    Robot.m_intake.runAll(m_speed);
  }
}
