/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ActivateIntakeCommand;
import frc.robot.commands.DeactivateIntakeCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.DriveTrainReversedCommand;
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

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final DriveTrainCommand driveTrainCommand = new DriveTrainCommand(driveTrainSubsystem, leftStick, rightStick);
  private final DriveTrainReversedCommand driveTrainReversedCommand = new DriveTrainReversedCommand(driveTrainSubsystem, leftStick, rightStick);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ActivateIntakeCommand activateIntakeCommand = new ActivateIntakeCommand(intakeSubsystem);
  private final DeactivateIntakeCommand deactivateIntakeCommand = new DeactivateIntakeCommand(intakeSubsystem);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveTrainSubsystem.setDefaultCommand(driveTrainCommand);

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
    new JoystickButton(leftStick, 1)
      .toggleWhenPressed(driveTrainReversedCommand);
    new JoystickButton(leftStick, 2)
      .whileHeld(activateIntakeCommand); // The correct method here might be .whileActive(), don't know...
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
