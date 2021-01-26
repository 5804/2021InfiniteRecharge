// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class ShooterSubsystem extends SubsystemBase {

  public WPI_TalonFX rightShooter;
  public WPI_TalonFX leftShooter;
  public WPI_TalonFX acceleratorMotors;

  

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    leftShooter = new WPI_TalonFX(5);
    rightShooter = new WPI_TalonFX(6);
    acceleratorMotors = new WPI_TalonFX(7);

    leftShooter.setInverted(true);

    rightShooter.follow(leftShooter);

    acceleratorMotors.enableVoltageCompensation(true);

    acceleratorMotors.configVoltageCompSaturation(12);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void startUpAccelerator() {
    acceleratorMotors.set(1.0);
  }

  public void setShooterSpeed() {
    leftShooter.set(0.5);
  }



}
