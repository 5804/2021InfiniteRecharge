/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  // Declaring intake solenoids
  public Solenoid intakeSolenoid1;

  // Declaring the motors in this subsystem
  public WPI_TalonFX outerIntakeMotor;
  public WPI_TalonFX innerIntakeMotor;

  public IntakeSubsystem() {
    // Inititalizing the outer intake solenoid
    intakeSolenoid1 = new Solenoid(1);

    // Initializng the three motors
    outerIntakeMotor = new WPI_TalonFX(10); 
    innerIntakeMotor = new WPI_TalonFX(9);

    // Enabling voltage compensation for the three motors
    outerIntakeMotor.enableVoltageCompensation(true);
    innerIntakeMotor.enableVoltageCompensation(true);

    // Configuring the volatge compSaturation for the motors
    outerIntakeMotor.configVoltageCompSaturation(12);
    innerIntakeMotor.configVoltageCompSaturation(12);
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void activateIntake() {
    // extend both pistons
    intakeSolenoid1.set(true);

    // spin the intake motor
    outerIntakeMotor.set(ControlMode.PercentOutput, Constants.OUTER_INTAKE_MOTOR_SPEED);
    innerIntakeMotor.set(ControlMode.PercentOutput, Constants.INNER_INTAKE_MOTOR_SPEED);
  }

  public void deactivateIntake() {
    // unextend both pistons
    intakeSolenoid1.set(false);

    // stop spinning the intake motor
    outerIntakeMotor.set(ControlMode.PercentOutput, 0);
    innerIntakeMotor.set(ControlMode.PercentOutput, 0);
  }

}
