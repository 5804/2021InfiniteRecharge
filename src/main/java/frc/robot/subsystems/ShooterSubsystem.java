// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class ShooterSubsystem extends SubsystemBase {

  // Declare motors for two shooter motors and one accelerator motor
  public WPI_TalonFX rightShooter;
  public WPI_TalonFX leftShooter;
  public WPI_TalonFX acceleratorMotors;

  public double initialTargetVelocity = 15000;
  public double sliderAddTargetVelocity;
  public double targetVelocity;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // Initialize three motors, two for the shooter and one for the accelerator
    leftShooter = new WPI_TalonFX(5);
    rightShooter = new WPI_TalonFX(6);
    acceleratorMotors = new WPI_TalonFX(7);

    // Configure the motors so that they spin in the intended direction
    leftShooter.setInverted(TalonFXInvertType.Clockwise);
    rightShooter.setInverted(TalonFXInvertType.CounterClockwise);
    rightShooter.follow(leftShooter);

    // Configuring the factory default for all motors
    leftShooter.configFactoryDefault();
    rightShooter.configFactoryDefault();
    acceleratorMotors.configFactoryDefault();

    // If voltage is below .1% of output, then don't run the leftShooter
    leftShooter.configNeutralDeadband(.001);

    // Setting brake mode to coast
    leftShooter.setNeutralMode(NeutralMode.Coast);
    rightShooter.setNeutralMode(NeutralMode.Coast);
    acceleratorMotors.setNeutralMode(NeutralMode.Coast);

    // 
    leftShooter.configNominalOutputForward(0, kTimeoutMs);
		leftShooter.configNominalOutputReverse(0, kTimeoutMs);
		leftShooter.configPeakOutputForward(1, kTimeoutMs);
    leftShooter.configPeakOutputReverse(-1, kTimeoutMs);
    
    // PID gains for velocity control for shooter
    leftShooter.config_kF(kPIDLoopIdx, kGains_Velocit.kF, kTimeoutMs);
		leftShooter.config_kP(kPIDLoopIdx, kGains_Velocit.kP, kTimeoutMs);
		leftShooter.config_kI(kPIDLoopIdx, kGains_Velocit.kI, kTimeoutMs);
		leftShooter.config_kD(kPIDLoopIdx, kGains_Velocit.kD, kTimeoutMs);

    // Further config for the accelerator motors
    // This sets the accelerator motors to 12 volts at 100%
    acceleratorMotors.enableVoltageCompensation(true);
    acceleratorMotors.configVoltageCompSaturation(12);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Velocity Setpoint", targetVelocity);
    SmartDashboard.putNumber("Actual Velocity", leftShooter.getSelectedSensorVelocity());

    // Add the sliderAddTargetVelocity number to the smartdashboard
    SmartDashboard.putNumber("Slider Add Target Velocity", sliderAddTargetVelocity);
  }
  public void startUpAccelerator() {
    acceleratorMotors.set(1.0);
  }

  public void setShooterSpeed(double slider) {
    // Getting the value of the slider and adjust the velocity of the shooter accordingly
    sliderAddTargetVelocity = (((slider*-1) + 1)/2)*5000;

    // Set the target velocity equal to 15000 plus the calculation above
    targetVelocity = initialTargetVelocity + sliderAddTargetVelocity;

    leftShooter.set(TalonFXControlMode.Velocity, targetVelocity);
    
  }

  public void stopShooter() {
    acceleratorMotors.set(0);
    leftShooter.set(0);
  }

}
