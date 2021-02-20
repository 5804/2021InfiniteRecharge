// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

// Distance Is In Meters
public class DriveForwardDistanceCommand extends CommandBase {

  private final DriveTrainSubsystem drive;
  public double driveDistance; 

  /** Creates a new DriveForwardDistance. */
  public DriveForwardDistanceCommand(DriveTrainSubsystem driveSub, double distance) {
    drive = driveSub;
    driveDistance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveForward(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveForward(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Average Sensor Position", drive.getAverageEncoderDistance());

    if (drive.getAverageEncoderDistance() >= driveDistance) {
      return true;
    }

    return false;
  }
}
