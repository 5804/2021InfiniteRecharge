// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchPhase2 extends SequentialCommandGroup {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final String pathToRun;

  /** Creates a new GalacticSearchPhase2. */
  public GalacticSearchPhase2(DriveTrainSubsystem dts, IntakeSubsystem is, LimelightSubsystem ls, String path) {

    driveTrainSubsystem = dts;
    intakeSubsystem = is;
    limelightSubsystem = ls;

    pathToRun = path;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }

  @Override
  public void initialize() {
    if (pathToRun.equals("red")) {
      if (limelightSubsystem.isRedPathA()) {
        addCommands(new GalacticSearchARed(driveTrainSubsystem, intakeSubsystem));
      } else {
        addCommands(new GalacticSearchBRed(driveTrainSubsystem, intakeSubsystem));
      }
    } else {
      if (limelightSubsystem.isBluePathA()) {
        addCommands(new GalacticSearchABlue(driveTrainSubsystem, intakeSubsystem));
      } else {
        addCommands(new GalacticSearchBBlue(driveTrainSubsystem, intakeSubsystem));
      }
    }

    super.initialize();
  }
}
