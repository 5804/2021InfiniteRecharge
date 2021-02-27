// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import static frc.robot.Constants.*;

import java.util.List;

public class SimplePathCommand extends RamseteCommand {

  private final DriveTrainSubsystem driveTrainSubsystem;
  
  /** Creates a new SimplePathCommand. */
  public SimplePathCommand(DriveTrainSubsystem driveTrainSubsystem, Trajectory exampleTrajectory) {

    super(
        exampleTrajectory,
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

    this.driveTrainSubsystem = driveTrainSubsystem;

    driveTrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveTrainSubsystem.tankDriveVolts(0.0, 0.0);
  }
}
