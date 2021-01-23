/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {

  public WPI_TalonFX conveyorMotor;

  // If Tof4 and Tof1 both see a ball, this will be set to true
  public boolean conveyorIsFull = false;

  /**
   * Creates a new ConveyorSubsystem.
   */
  public ConveyorSubsystem() {
    // Initializing the conveyor motor and configuring it
    conveyorMotor = new WPI_TalonFX(8);

    conveyorMotor.enableVoltageCompensation(true);
    conveyorMotor.configVoltageCompSaturation(12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void activateConveyor() {
    // Checks if the conveyor is full and does not run the conveyor motor if it is full
    if (!conveyorIsFull) {
      conveyorMotor.set(ControlMode.PercentOutput, Constants.CONVEYOR_MOTOR_SPEED);
    }
  }

  public void deactivateConveyor() {
    conveyorMotor.set(ControlMode.PercentOutput, 0);
  }
}
