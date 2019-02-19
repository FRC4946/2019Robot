/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivetrain.SetDriveTrain;
import frc.robot.commands.intake.SetIntakeSpeed;

public class ClimbOnPlatform extends CommandGroup {
  
  /**
   * Add your docs here.
   */
  public ClimbOnPlatform(double boardingSpeed, double height) {

    addSequential(new SetClimberHeight(height));
    addParallel(new SetIntakeSpeed(boardingSpeed), 3.0);
    addSequential(new SetDriveTrain(boardingSpeed), 3.0);
  }
}
