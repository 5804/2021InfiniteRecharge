// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForwardDistanceCommand;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndShootCommandGroup extends SequentialCommandGroup {
  /** Creates a new DriveAndShootCommand. */
  public DriveAndShootCommandGroup(DriveTrainSubsystem dts, ShooterSubsystem shootSub, Joystick left, LimelightSubsystem ls) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveForwardDistanceCommand(dts, 1.0),
                new AimAndRunShooterMotorCommandGroup(shootSub, left, ls, dts));
  }
}
