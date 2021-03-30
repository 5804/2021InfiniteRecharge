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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ResetOdometryCommand;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PracticeBounce extends SequentialCommandGroup {
  /** Creates a new PracticeBounce. */
  public PracticeBounce(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem) {
    //String trajectoryJSON = "paths/output/bounce1.wpilib.json";
    String trajectoryJSON = "paths/output/bounceP1.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand bouncePath1 = new RamseteCommand(
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

    //trajectoryJSON = "paths/output/bounce2.wpilib.json";
    trajectoryJSON = "paths/output/bounceP2.wpilib.json";
    Trajectory trajectory1 = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand bouncePath2 = new RamseteCommand(
      trajectory1,
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

      //driveTrainSubsystem.resetOdometry(trajectory.getInitialPose());

    //trajectoryJSON = "paths/output/bounce3.wpilib.json";
    trajectoryJSON = "paths/output/bounceP3path.wpilib.json";
    Trajectory trajectory2 = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand bouncePath3 = new RamseteCommand(
      trajectory2,
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

      //driveTrainSubsystem.resetOdometry(trajectory.getInitialPose());

    //trajectoryJSON = "paths/output/bounce5.wpilib.json";
    trajectoryJSON = "paths/output/bounceP4.wpilib.json";
    Trajectory trajectoryFinal = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectoryFinal = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand bouncePath5 = new RamseteCommand(
      trajectoryFinal,
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
    
      //driveTrainSubsystem.resetOdometry(trajectoryFinal.getInitialPose());
      
      ResetOdometryCommand resetOdometryCommand = new ResetOdometryCommand(driveTrainSubsystem, trajectory);

      ActivateIntakeCommand activateIntakeCommand = new ActivateIntakeCommand(intakeSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(resetOdometryCommand,
    bouncePath1, bouncePath2, bouncePath3, bouncePath5, activateIntakeCommand);
  }
}
