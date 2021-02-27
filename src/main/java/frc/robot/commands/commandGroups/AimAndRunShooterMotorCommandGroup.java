// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlignToGoalWithLimelightCommand;
import frc.robot.commands.RunShooterMotor;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndRunShooterMotorCommandGroup extends ParallelCommandGroup {

  // private final LimelightSubsystem limelightSubsystem;
  // private final DriveTrainSubsystem driveTrainSubsystem;
  // private final ShooterSubsystem shoot;
  // private final Joystick leftStick;

  /** Creates a new AimAndShooterMotor. */
  public AimAndRunShooterMotorCommandGroup(ShooterSubsystem shootSub, Joystick left, LimelightSubsystem ls, DriveTrainSubsystem dts) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AlignToGoalWithLimelightCommand(ls, dts), 
                new RunShooterMotor(shootSub, left));
  }
}
