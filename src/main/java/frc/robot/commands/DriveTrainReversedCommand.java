/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainReversedCommand extends CommandBase {

  private final DriveTrainSubsystem drive; 
  
  private final Joystick leftStick;
  private final Joystick rightStick;

  /**
   * Creates a new DriveTrainReversedCommand.
   */
  public DriveTrainReversedCommand(DriveTrainSubsystem driveSub, Joystick left, Joystick right) {
    drive = driveSub;
    leftStick = left;
    rightStick = right;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The joystick values are turned negative in the method driveWithJoystickReversed() in DriveTrainSubsystem.java
    drive.driveWithJoystickReversed(leftStick.getY(), rightStick.getY());
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
