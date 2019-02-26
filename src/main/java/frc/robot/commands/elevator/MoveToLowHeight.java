/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConstants;
import frc.robot.commands.elevator.MoveToHeight;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class MoveToLowHeight extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MoveToLowHeight() { 
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT + 0.2, 0.2));
    addSequential(new SetIntakePos(RobotConstants.INTAKE_POT_BALL_HEIGHT, 0.3));
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_AT_MIN + 0.1, 0.4));
  }
}
