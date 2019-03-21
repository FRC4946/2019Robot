/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConstants;
import frc.robot.commands.grabber.SetGrabber;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class MoveToLevel2Rocket extends CommandGroup {
  public MoveToLevel2Rocket() {
    addParallel(new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.25), 1.5);
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_LEVEL_2_ROCKET + 0.3, 0.4), 2.0);
    addSequential(new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_HATCH, 0.8), 0.8); 
  }
}
