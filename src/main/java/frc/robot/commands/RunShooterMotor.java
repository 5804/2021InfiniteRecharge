// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterMotor extends CommandBase {

  private final ShooterSubsystem shoot;

  private final Joystick leftStick;
  /** Creates a new ShooterCommand. */
  public RunShooterMotor(ShooterSubsystem shootSub, Joystick left) {
    shoot = shootSub;

    leftStick = left;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Spin the accelerator and spin the shooter until the velocity is at target value
    shoot.startUpAccelerator();

    // Get the value of the slider and pass it into the setShooterSpeed method of the shooter subsystem
    shoot.setShooterSpeed(leftStick.getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop both the accelerator and the shooter
    shoot.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
