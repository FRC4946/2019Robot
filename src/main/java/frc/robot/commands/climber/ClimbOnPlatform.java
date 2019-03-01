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
   * Add your docs here.
   */
  public ClimbOnPlatform() {

    addSequential(new SetIntakePos(RobotConstants.INTAKE_POT_DOWN, 0.3), 2.0);
    addSequential(new SetIntakeSpeed(0.99), 6.0);
    addSequential(new SetDriveTrain(0.15), 6.0);
    addSequential(new SetDriveTrain(0));
    addSequential(new SetIntakeSpeed(0));
  }
}
