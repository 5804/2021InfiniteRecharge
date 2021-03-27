/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.commandGroups.AimAndRunShooterMotorCommandGroup;
import frc.robot.commands.commandGroups.DriveForward5Feet;
import frc.robot.commands.commandGroups.Bounce;
import frc.robot.commands.commandGroups.DriveAndShootCommandGroup;
import frc.robot.commands.commandGroups.GalacticSearchABlue;
import frc.robot.commands.commandGroups.GalacticSearchARed;
import frc.robot.commands.commandGroups.GalacticSearchBBlue;
import frc.robot.commands.commandGroups.GalacticSearchBRed;
import frc.robot.commands.commandGroups.PracticeBounce;
import frc.robot.commands.commandGroups.GalacticSearchPhase1;
import frc.robot.commands.commandGroups.Slalom;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.LimelightSubsystem;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public Joystick leftStick = new Joystick(0);
  public Joystick rightStick = new Joystick(1);

  // The robot's subsystems and commands are defined here
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Drivetrain subsystem and commands
  public final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  // private final DriveTrainCommand driveTrainCommand = new DriveTrainCommand(driveTrainSubsystem);
  // private final DriveTrainReversedCommand driveTrainReversedCommand = new DriveTrainReversedCommand(driveTrainSubsystem);
  private final DriveWithJoysticksCommand driveWithJoysticksCommand = new DriveWithJoysticksCommand(driveTrainSubsystem, leftStick, rightStick);
  private final DriveWithArcadeCommand driveWithArcadeCommand = new DriveWithArcadeCommand(driveTrainSubsystem, rightStick);

  // Intake subsystem and commands
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final ActivateIntakeCommand activateIntakeCommand = new ActivateIntakeCommand(intakeSubsystem);
  private final DeactivateIntakeCommand deactivateIntakeCommand = new DeactivateIntakeCommand(intakeSubsystem);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShooterIdleCommand shooterIdleCommand = new ShooterIdleCommand(shooterSubsystem);
  private final ShooterDashVelocityCommand shooterDashVelocityCommand = new ShooterDashVelocityCommand(shooterSubsystem);

  private final DriveAndShootCommandGroup autonomousFromLineCommand = new DriveAndShootCommandGroup(driveTrainSubsystem, shooterSubsystem, leftStick, limelightSubsystem);

  private final FireIntakeCommand fireCommand = new FireIntakeCommand(intakeSubsystem);

  private final GalacticSearchBBlue galacticSearchBBlue = new GalacticSearchBBlue(driveTrainSubsystem, intakeSubsystem);
  private final GalacticSearchBRed galacticSearchBRed = new GalacticSearchBRed(driveTrainSubsystem, intakeSubsystem);
  private final GalacticSearchARed galacticSearchARed = new GalacticSearchARed(driveTrainSubsystem, intakeSubsystem);
  private final GalacticSearchABlue galacticSearchABlue = new GalacticSearchABlue(driveTrainSubsystem, intakeSubsystem);

  private final GalacticSearchPhase1 galacticSearchPhase1 = new GalacticSearchPhase1(driveTrainSubsystem, intakeSubsystem, limelightSubsystem);

  private final Slalom slalom = new Slalom(driveTrainSubsystem, intakeSubsystem);

  private final Bounce bounce = new Bounce(driveTrainSubsystem, intakeSubsystem);

  private final DriveForward5Feet barrelRacingPath = new DriveForward5Feet(driveTrainSubsystem, intakeSubsystem);

  private final PracticeBounce practice = new PracticeBounce(driveTrainSubsystem, intakeSubsystem);

  SendableChooser<Command> sendableChooser = new SendableChooser<>();

  // Ramsete Command Setup
  DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(KS, KV, KA), K_DRIVE_KINEMATICS, 10);

  TrajectoryConfig config = new TrajectoryConfig(
    kMaxSpeedMetersPerSecond,
    kMaxAccelerationMetersPerSecondSquared)
  // Add kinematics to ensure max speed is actually obeyed
  .setKinematics(K_DRIVE_KINEMATICS)
  // Apply the voltage constraint
  .addConstraint(autoVoltageConstraint);


  //String bluePathGalactic = "paths/bluePathGalacticPathB.wpilib.json";
  //Trajectory trajectory = getTrajectoryFromPath(bluePathGalactic);
  //String trajectoryJSON = "paths/bluePathGalacticPathB";


  /*
  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
        new Translation2d(1.524, 0),
        new Translation2d(3.05, -0.76),
        new Translation2d(3.5, 1.7)
    ),
    new Pose2d(7.0, 1.0, new Rotation2d(0)),
    // Pass config
    config
);
*/


  /*private final SimplePathCommand simplePathCommand = new SimplePathCommand(
    driveTrainSubsystem,
    trajectory
  );
  */
  
  // All joystick buttons are defined here
  JoystickButton leftTrigger = new JoystickButton(leftStick, 1);
  JoystickButton leftThumbMain = new JoystickButton(leftStick, 2);
  JoystickButton leftThumbLeft = new JoystickButton(leftStick, 3);
  JoystickButton leftThumbRight = new JoystickButton(leftStick, 4);
  JoystickButton leftRightArrayTR = new JoystickButton(leftStick, 5);
  JoystickButton leftRightArrayTM = new JoystickButton(leftStick, 6);
  JoystickButton leftRightArrayTL = new JoystickButton(leftStick, 7);
  JoystickButton leftRightArrayBL = new JoystickButton(leftStick, 8);
  JoystickButton leftRightArrayBM = new JoystickButton(leftStick, 9);
  JoystickButton leftRightArrayBR = new JoystickButton(leftStick, 10);
  JoystickButton leftLeftArrayTL = new JoystickButton(leftStick, 11);
  JoystickButton leftLeftArrayTM = new JoystickButton(leftStick, 12);
  JoystickButton leftLeftArrayTR = new JoystickButton(leftStick, 13);
  JoystickButton leftLeftArrayBR = new JoystickButton(leftStick, 14);
  JoystickButton leftLeftArrayBM = new JoystickButton(leftStick, 15);
  JoystickButton leftLeftArrayBL = new JoystickButton(leftStick, 16);

  JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
  JoystickButton rightThumbMain = new JoystickButton(rightStick, 2);
  JoystickButton rightThumbLeft = new JoystickButton(rightStick, 3);
  JoystickButton rightThumbRight = new JoystickButton(rightStick, 4);
  JoystickButton rightRightArrayTR = new JoystickButton(rightStick, 5);
  JoystickButton rightRightArrayTM = new JoystickButton(rightStick, 6);
  JoystickButton rightRightArrayTL = new JoystickButton(rightStick, 7);
  JoystickButton rightRightArrayBL = new JoystickButton(rightStick, 8);
  JoystickButton rightRightArrayBM = new JoystickButton(rightStick, 9);
  JoystickButton rightRightArrayBR = new JoystickButton(rightStick, 10);
  JoystickButton rightLeftArrayTL = new JoystickButton(rightStick, 11);
  JoystickButton rightLeftArrayTM = new JoystickButton(rightStick, 12);
  JoystickButton rightLeftArrayTR = new JoystickButton(rightStick, 13);
  JoystickButton rightLeftArrayBR = new JoystickButton(rightStick, 14);
  JoystickButton rightLeftArrayBM = new JoystickButton(rightStick, 15);
  JoystickButton rightLeftArrayBL = new JoystickButton(rightStick, 16);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    driveTrainSubsystem.setDefaultCommand(driveWithArcadeCommand);
 
    // if there are no commands running on the intake, the intake will be deactivated
    intakeSubsystem.setDefaultCommand(deactivateIntakeCommand);
    shooterSubsystem.setDefaultCommand(shooterIdleCommand);

    //sendableChooser.setDefaultOption("Simple Path", simplePathCommand);
    sendableChooser.setDefaultOption("Galactic Search - Path B - Blue", galacticSearchBBlue);
    sendableChooser.addOption("Galactic Search - Path B - Red", galacticSearchBRed);
    sendableChooser.addOption("Slalom", slalom);
    sendableChooser.addOption("Bounce", bounce);
    sendableChooser.addOption("Barrel Racing", barrelRacingPath);
    sendableChooser.setDefaultOption("Galactic Search - Path A - Blue", galacticSearchABlue);
    sendableChooser.addOption("Galactic Search - Path A - Red", galacticSearchARed);
    sendableChooser.addOption("Practice Bounce", practice);
    sendableChooser.addOption("Fire", fireCommand);

    sendableChooser.addOption("Galactic Search", galacticSearchPhase1);

    SmartDashboard.putData(sendableChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Leftstick button mappings
    rightThumbRight
      .whenPressed(new DriveTrainCommand(driveTrainSubsystem));
    rightThumbLeft
      .whenPressed(new DriveTrainReversedCommand(driveTrainSubsystem));

    leftTrigger
      .whileHeld(new FireIntakeCommand(intakeSubsystem));
    // Rightstick button mappings
    rightTrigger
      .whileHeld(new ActivateIntakeCommand(intakeSubsystem));
      
    //shooter is the rightthumbmain, to actually fire, hold the righttrigger
    leftThumbMain
      .whileHeld(new AimAndRunShooterMotorCommandGroup(shooterSubsystem, leftStick, limelightSubsystem, driveTrainSubsystem));
    leftThumbRight
      .whileHeld(new ShooterDashVelocityCommand(shooterSubsystem));

    rightThumbMain
      .whenPressed(new ResetEncoderCommand(driveTrainSubsystem));

    leftLeftArrayBL
      .whenPressed(new DriveAndShootCommandGroup(driveTrainSubsystem, shooterSubsystem, leftStick, limelightSubsystem));
    
    /*leftLeftArrayBR
      .whenPressed(simplePathCommand);
      */
  }

  public Trajectory getTrajectoryFromPath(String path) {
    Trajectory trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        return trajectory;
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        return null;
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    /*String trajectoryJSON = "paths/galacticPathBBlue.wpilib.json";
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

      // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrainSubsystem.tankDriveVolts(0, 0));*/
    return sendableChooser.getSelected();
  }
}