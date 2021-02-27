/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.commandGroups.AimAndRunShooterMotorCommandGroup;
import frc.robot.commands.commandGroups.DriveAndShootCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.LimelightSubsystem;

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
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
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

  SendableChooser<Command> sendableChooser = new SendableChooser<>();
  
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

    sendableChooser.setDefaultOption("Drive and Shooting", autonomousFromLineCommand);
    sendableChooser.addOption("Fire", fireCommand);

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
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
