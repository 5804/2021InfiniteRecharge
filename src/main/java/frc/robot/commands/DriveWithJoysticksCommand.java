/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveWithJoysticksCommand extends CommandBase {

  private final DriveTrainSubsystem drive; 
  
  private final Joystick leftStick;
  private final Joystick rightStick;


  /**
   * Creates a new DriveWithJoysticks.
   */
  public DriveWithJoysticksCommand(DriveTrainSubsystem driveSub, Joystick left, Joystick right) {

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
    drive.driveWithJoystick(leftStick.getY(), rightStick.getY());
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
