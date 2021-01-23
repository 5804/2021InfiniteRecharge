/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class UpdateToFsCommand extends CommandBase {

  private final ConveyorSubsystem conveyorSubsystem;

  public TimeOfFlight timeOfFlight1;
  public TimeOfFlight timeOfFlight2;
  public TimeOfFlight timeOfFlight3;
  public TimeOfFlight timeOfFlight4;

  /**
   * Creates a new UpdateToFsCommand.
   */
  public UpdateToFsCommand(ConveyorSubsystem cs) {
    conveyorSubsystem = cs;
    addRequirements(conveyorSubsystem);

    timeOfFlight1 = new TimeOfFlight(1);
    timeOfFlight2 = new TimeOfFlight(2);
    timeOfFlight3 = new TimeOfFlight(3);
    timeOfFlight4 = new TimeOfFlight(4);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeOfFlight1.setRangingMode(RangingMode.Short, Constants.TOF_SAMPLE_TIME);
    timeOfFlight2.setRangingMode(RangingMode.Short, Constants.TOF_SAMPLE_TIME);
    timeOfFlight3.setRangingMode(RangingMode.Short, Constants.TOF_SAMPLE_TIME);
    timeOfFlight4.setRangingMode(RangingMode.Short, Constants.TOF_SAMPLE_TIME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (getTimeOfFlight1Stat() && getTimeOfFlight4Stat()) {
      conveyorSubsystem.deactivateConveyor();
    }
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

  public boolean getTimeOfFlight1Stat() {
    if (timeOfFlight1.getRange() < 110) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getTimeOfFlight2Stat() {
    if (timeOfFlight2.getRange() < 100) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getTimeOfFlight3Stat() {
    if (timeOfFlight3.getRange() < 100) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getTimeOfFlight4Stat() {
    if (timeOfFlight4.getRange() < 110) {
      return true;
    } else {
      return false;
    }
  }
}
