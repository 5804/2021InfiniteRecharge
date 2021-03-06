/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
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
    // if either tof1 or tof4 dont see something
    if (!intakeSubsystem.getTimeOfFlight1Stat() || !intakeSubsystem.getTimeOfFlight4Stat()) {
      // if either tof2 or tof3 see something
      if (intakeSubsystem.getTimeOfFlight2Stat() || intakeSubsystem.getTimeOfFlight3Stat()) {
        intakeSubsystem.activateIntake(INNER_INTAKE_MOTOR_SPEED, CONVEYOR_MOTOR_SPEED);
      } else {
        // if either tof2 or tof3 dont see something, then run outer, inner, but not conveyor
        intakeSubsystem.activateIntake(INNER_INTAKE_MOTOR_SPEED, STOP_MOTOR);
      }
    } else {
      // if tof1 and tof4 see something, deactivate the intake
      intakeSubsystem.deactivateIntake();
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
