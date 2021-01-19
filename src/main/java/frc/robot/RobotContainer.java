/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.DeactivateIntakeCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.DriveTrainReversedCommand;
import frc.robot.commands.DriveWithJoysticksCommand;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

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
  // private final DriveTrainCommand driveTrainCommand = new DriveTrainCommand(driveTrainSubsystem);
  // private final DriveTrainReversedCommand driveTrainReversedCommand = new DriveTrainReversedCommand(driveTrainSubsystem);
  private final DriveWithJoysticksCommand driveWithJoysticksCommand = new DriveWithJoysticksCommand(driveTrainSubsystem, leftStick, rightStick);

  // Intake subsystem and commands
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final ActivateIntakeCommand activateIntakeCommand = new ActivateIntakeCommand(intakeSubsystem);
  private final DeactivateIntakeCommand deactivateIntakeCommand = new DeactivateIntakeCommand(intakeSubsystem);

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

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    driveTrainSubsystem.setDefaultCommand(driveWithJoysticksCommand);

    // if there are no commands running on the shooter, the shooter will be deactivated
    intakeSubsystem.setDefaultCommand(deactivateIntakeCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // make normal and reversed two seperate buttons, so that the drivers know what mode they are in
    // button 1: driveTrainCommand (set the isReversed in the driveTrainSubsystem to false)
    // button 2: driveTrainReversedCommand (set the isReversed in the driveTrainSubsystem to true)
    leftThumbMain
      .whenPressed(new DriveTrainCommand(driveTrainSubsystem));
    leftThumbRight
      .whenPressed(new DriveTrainReversedCommand(driveTrainSubsystem));
    leftThumbLeft
      .whileHeld(new ActivateIntakeCommand(intakeSubsystem)); // The correct method here might be .whileActive(), don't know...
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
