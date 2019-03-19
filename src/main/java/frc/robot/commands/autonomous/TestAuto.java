/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivetrain.DriveStraightPID;
import frc.robot.commands.drivetrain.TargetLine;
import frc.robot.commands.drivetrain.TurnPID;
import frc.robot.commands.limelight.SetLimelightLED;

public class TestAuto extends CommandGroup {

  /**
   * Test autonomus code
   */
  public TestAuto() {
    /*
    addSequential(new SetLimelightLED(false));
    addSequential(new TurnPID(180, 0.3, false));
    addSequential(new DriveStraightPID(60));
    addSequential(new TurnPID(-90, 0.3, false));
    addSequential(new DriveStraightPID(-24));*/
  }
}
