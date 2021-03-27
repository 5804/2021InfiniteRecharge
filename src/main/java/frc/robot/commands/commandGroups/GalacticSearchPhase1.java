// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchPhase1 extends SequentialCommandGroup {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  /** Creates a new GalacticSearchCommandGroup. */
  public GalacticSearchPhase1(DriveTrainSubsystem dts, IntakeSubsystem is, LimelightSubsystem ls) {

    driveTrainSubsystem = dts;
    intakeSubsystem = is;
    limelightSubsystem = ls;

    // if isRedPath():
    //  drive forward 5 feet
    //  if isRedPathA:
    //    use pathweaver to run the rest of red path A
    //  else:
    //    use pathweaver to run the rest of red path B
    // else: 
    //  drive forward 12 feet
    //  if isBluePathA:
    //    use pathweaver to run the rest of blue path A
    //  else:
    //    use pathweaver to run the rest of blue path B

    // SequentialCommandGroup(command1, command2)
  }

  @Override
  public void initialize() {
    if (limelightSubsystem.isRedPath()) {
      // drive forward 5 feet
      addCommands(new DriveForward5Feet(driveTrainSubsystem, intakeSubsystem));
    } else {
      // drive forward 12 feet
      addCommands(new DriveForward12Feet(driveTrainSubsystem, intakeSubsystem));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
