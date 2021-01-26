/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ActivateIntakeCommand extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;

  /**
   * Creates a new ActivateIntakeCommand.
   */
  public ActivateIntakeCommand(IntakeSubsystem is) {
    intakeSubsystem = is;
    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeSubsystem.timeOfFlight1.getRange() >= 110 || intakeSubsystem.timeOfFlight4.getRange() >= 110) {
      if (intakeSubsystem.timeOfFlight2.getRange() < 100 || intakeSubsystem.timeOfFlight3.getRange() < 100) {
        intakeSubsystem.activateIntake(0.65, 1, 0.6);
      } else {
        intakeSubsystem.activateIntake(0.65, 1, 0);
      }
    } else {
      intakeSubsystem.activateIntake(0.65, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.deactivateIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
