/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConstants;
import frc.robot.commands.grabberarm.SetArmToPos;

public class MoveToLowHeight extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MoveToLowHeight() { 
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT + 0.3, 0.4), 2.0);
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_AT_MIN + 0.05, 0.4), 4.0); 
  }
}