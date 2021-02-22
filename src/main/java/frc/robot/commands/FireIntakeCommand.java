// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class FireCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  /** Creates a new FireCommand. */
  public FireCommand(IntakeSubsystem is) {
    intakeSubsystem = is;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run only the conveyor and the inner intake
    intakeSubsystem.fire();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the conveyor and the inner intake once the lefttrigger is released
    intakeSubsystem.fireDeactivate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
