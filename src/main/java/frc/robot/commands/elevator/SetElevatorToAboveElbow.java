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
import frc.robot.commands.intakeelbow.SetIntakePos;

public class SetElevatorToAboveElbow extends CommandGroup {
  /**
   *  An autonomus command to toggle the elbows.
   */
  public SetElevatorToAboveElbow() {
    //addSequential(new SetIntakePos(RobotConstants.INTAKE_POT_DOWN, 0.25), 1.5);
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT + 0.5, 0.6), 4.0); 
    addSequential(new WaitCommand(0.5));
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW + 0.05, 0.4), 2.0);
    //addSequential(new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.25), 1.5);
  }
}
