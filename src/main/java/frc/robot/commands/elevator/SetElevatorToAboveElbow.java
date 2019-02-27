/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotConstants;

public class SetElevatorToAboveElbow extends CommandGroup {
  /**
   * Add your docs here.
   */
  public SetElevatorToAboveElbow() {
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT, 0.4));
    addSequential(new WaitCommand(1.0)); 
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW, 0.1));
  }
}
