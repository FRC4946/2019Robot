/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConstants;
import frc.robot.commands.drivetrain.SetDriveTrain;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class ClimbOnPlatform extends CommandGroup {
  
  /**
   * Autonomus code to make the climber work without human imput.
   */
  public ClimbOnPlatform() {

    addParallel(new ClimbToPos(0)); //sync the legs

    //raise height
    addSequential(new ClimbToPos(RobotConstants.LOWER_PLATFORM_HEIGHT));

    //move forward and raise the front legs up
    addSequential(new SetIntakePos(RobotConstants.INTAKE_POT_DOWN, 0.3), 2.0);
    addParallel(new SetIntakeSpeed(0.99), 2.0);
    addSequential(new SetDriveTrain(0.1), 2.0);
    addSequential(new ClimbToPosFront(0), 2.0);

    //move forward and raise the back legs up
    addParallel(new SetDriveTrain(0.99), 2.0);
    addSequential(new SetIntakeSpeed(0.1), 2.0);
    addSequential(new ClimbToPosBack(0), 2.0);

    //move forward and complete the climb
    addParallel(new SetDriveTrain(0.99), 1.0);
    addSequential(new SetIntakeSpeed(0.1), 1.0);

  }
}
