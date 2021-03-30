// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class ResetOdometryCommand extends CommandBase {
  
  private final DriveTrainSubsystem drive;
  Trajectory trajectory;

  /** Creates a new ResetOdometryCommand. */
  public ResetOdometryCommand(DriveTrainSubsystem driveSub, Trajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = driveSub;
    trajectory = traj;

    addRequirements(drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.resetOdometry(trajectory.getInitialPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
