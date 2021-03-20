// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Bounce extends SequentialCommandGroup {
  /** Creates a new Bounce. */
  public Bounce(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem) {
    String trajectoryJSON = "paths/Bounce.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      driveTrainSubsystem::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(KS,
                                KV,
                                KA),
                                K_DRIVE_KINEMATICS,
      driveTrainSubsystem::getWheelSpeeds,
      new PIDController(KP, 0, 0),
      new PIDController(KP, 0, 0),
      // RamseteCommand passes volts to the callback
      driveTrainSubsystem::tankDriveVolts,
      driveTrainSubsystem
      );

      driveTrainSubsystem.resetOdometry(trajectory.getInitialPose());

      
      ActivateIntakeCommand activateIntakeCommand = new ActivateIntakeCommand(intakeSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(ramseteCommand, activateIntakeCommand);
  }
}
