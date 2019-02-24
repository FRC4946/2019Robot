/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeelbow;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConstants;
import frc.robot.commands.elevator.MoveToHeight;

public class SetIntakeStage extends CommandGroup {
  /**
   * Add your docs here.
   */
  public SetIntakeStage(double intakeStage) {
    if (!(intakeStage == RobotConstants.INTAKE_POT_BALL_HEIGHT || intakeStage == RobotConstants.INTAKE_POT_DOWN || intakeStage == RobotConstants.INTAKE_POT_UP)) {
      intakeStage = RobotConstants.INTAKE_POT_BALL_HEIGHT;
    }
    if (intakeStage == RobotConstants.INTAKE_POT_UP) {
      addSequential(new MoveToHeight(RobotConstants.ELEVATOR_CONFLICT_HEIGHT, 0.5));
    }
    addSequential(new SetIntakePos(intakeStage, 0.5));
  }
}
