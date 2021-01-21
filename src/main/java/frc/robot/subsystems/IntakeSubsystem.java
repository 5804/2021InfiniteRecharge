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
  public WPI_TalonFX conveyorMotor;

  // Declaring four time of flights
  public TimeOfFlight timeOfFlight1;
  public TimeOfFlight timeOfFlight2;
  public TimeOfFlight timeOfFlight3;
  public TimeOfFlight timeOfFlight4;

  public IntakeSubsystem() {
    // Inititalizing the outer intake solenoid
    intakeSolenoid1 = new Solenoid(1);

    // Initializng the three motors
    outerIntakeMotor = new WPI_TalonFX(10); 
    innerIntakeMotor = new WPI_TalonFX(9);
    conveyorMotor = new WPI_TalonFX(8);

    // Enabling voltage compensation for the three motors
    outerIntakeMotor.enableVoltageCompensation(true);
    innerIntakeMotor.enableVoltageCompensation(true);
    conveyorMotor.enableVoltageCompensation(true);

    // Configuring the volatge compSaturation for the motors
    outerIntakeMotor.configVoltageCompSaturation(12);
    innerIntakeMotor.configVoltageCompSaturation(12);
    conveyorMotor.configVoltageCompSaturation(12);

    // Initializing four time of flights
    timeOfFlight1 = new TimeOfFlight(1);
    timeOfFlight2 = new TimeOfFlight(2);
    timeOfFlight3 = new TimeOfFlight(3);
    timeOfFlight4 = new TimeOfFlight(4);

    // Setting the ranging mode for all of the time of flights to short
    timeOfFlight1.setRangingMode(RangingMode.Short, 25);
    timeOfFlight2.setRangingMode(RangingMode.Short, 25);
    timeOfFlight3.setRangingMode(RangingMode.Short, 25);
    timeOfFlight4.setRangingMode(RangingMode.Short, 25);
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

  // If time of flight 1 and 4:
  // the thing is full and the conveyor will not move

}
