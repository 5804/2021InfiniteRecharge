/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class ActivateConveyorCommand extends CommandBase {

  private final ConveyorSubsystem conveyorSubsystem;

  /**
   * Creates a new ActivateConveyorCommand.
   */
  public ActivateConveyorCommand(ConveyorSubsystem cs) {
    conveyorSubsystem = cs;
    addRequirements(conveyorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The logic checking if the conveyor is full is in the conveyor subsystem
    if (conveyorSubsystem.getTimeOfFlight2Stat() || conveyorSubsystem.getTimeOfFlight3Stat()) {
      conveyorSubsystem.activateConveyor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyorSubsystem.deactivateConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
