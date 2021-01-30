/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.*;

public class RotateToTargetCommand extends CommandBase {

  private final VisionSubsystem visionSubsystem;
  private final DriveTrainSubsystem driveTrainSubsystem;
  private int stableDone;

  /**
   * Creates a new RotateToTargetCommand.
   */
  public RotateToTargetCommand(VisionSubsystem vision, DriveTrainSubsystem dts) {
    visionSubsystem = vision;
    driveTrainSubsystem = dts;
    addRequirements(visionSubsystem, driveTrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stableDone = 0;
    // driveTrainSubsystem gyro needs to reset
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = 0;

    double tx = visionSubsystem.getTX();
    if (tx < VISION_TX_TOLERANCE && tx > -VISION_TX_TOLERANCE) {
      right = 0;
      stableDone++;
    }else {
      stableDone = 0;
      right = tx * VISION_STEER_KP;
      if (right > 0 && right < MIN_TURN_OUTPUT) {
        right = MIN_TURN_OUTPUT;
      } else if (right < 0 && right > -MIN_TURN_OUTPUT) {
        right = -MIN_TURN_OUTPUT;
      }
    }

    driveTrainSubsystem.driveWithArcade(0, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stableDone >= 20;
  }
}
