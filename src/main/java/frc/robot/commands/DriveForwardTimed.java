/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveForwardTimed extends CommandBase {

  private final DriveTrainSubsystem driveTrainSubsystem;
  private boolean finish = false;
  Timer timer;
  /**
   * Creates a new DriveForwardTimed.
   */
  public DriveForwardTimed(DriveTrainSubsystem dts) {
    driveTrainSubsystem = dts;
    addRequirements(driveTrainSubsystem);

    timer = new Timer();
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while (timer.get() < Constants.DRIVE_FORWARD_TIME) {
      driveTrainSubsystem.driveForward(Constants.AUTONOMOUS_SPEED, Constants.AUTONOMOUS_SPEED);
    }

    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
