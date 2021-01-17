/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public Solenoid intakeSolenoid1;
  public Solenoid intakeSolenoid2;

  public WPI_TalonFX intakeMotor;

  public IntakeSubsystem() {
    intakeSolenoid1 = new Solenoid(0);
    intakeSolenoid2 = new Solenoid(1);

    intakeMotor = new WPI_TalonFX(5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void activateIntake() {
    // extend both pistons
    intakeSolenoid1.set(true);
    intakeSolenoid1.set(true);

    // spin the intake motor
    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_MOTOR_SPEED);
  }

  public void deactivateIntake() {
    // unextend both pistons
    intakeSolenoid1.set(false);
    intakeSolenoid2.set(false);

    // stop spinning the intake motor
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
