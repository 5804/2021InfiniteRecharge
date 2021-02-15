/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignToGoalWithLimelightCommand extends CommandBase {

  private final LimelightSubsystem limelightSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private double steerCommand;

  /**
   * Creates a new AlignWithLimelightCommand.
   */
  public AlignToGoalWithLimelightCommand(LimelightSubsystem ls, DriveTrainSubsystem dts) {

    limelightSubsystem = ls;
    driveTrainSubsystem = dts;
    addRequirements(limelightSubsystem, driveTrainSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    steerCommand = limelightSubsystem.getSteeringValue();
    driveTrainSubsystem.driveWithArcade(0, steerCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
