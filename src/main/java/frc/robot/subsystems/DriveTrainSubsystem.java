/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class DriveTrainSubsystem extends SubsystemBase {
  
  public WPI_TalonFX rightMain;
  public WPI_TalonFX rightFollow;
  public WPI_TalonFX leftMain;
  public WPI_TalonFX leftFollow;

  public PigeonIMU pigeon;

  public DifferentialDrive twoMotorDrive;

  private final DifferentialDriveOdometry odometry;
  // public Pose2d pose;

  // SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.ks, Constants.kv); // Doesn't accept ka?
  // DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DRIVE_TRAIN_WIDTH);

  // // PIDController???
  // PIDController leftPIDController = new PIDController(0.1, 0.0, 0.0);
  // PIDController rightPIDController = new PIDController(0.1, 0.0, 0.0);

  public boolean isReversed = false;
  
  /**
   * Creates a new DriveTrainSubsystem.
   */
  public DriveTrainSubsystem() {
    leftMain = new WPI_TalonFX(1);
    leftFollow = new WPI_TalonFX(2);
    rightMain = new WPI_TalonFX(3);
    rightFollow = new WPI_TalonFX(4);

    pigeon = new PigeonIMU(0);

    leftMain.setInverted(true);
    leftFollow.setInverted(true); 
    rightMain.setInverted(true);
    rightFollow.setInverted(true);

    leftFollow.follow(leftMain);
    rightFollow.follow(rightMain);

    leftMain.setNeutralMode(NeutralMode.Brake);
    rightMain.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);

    twoMotorDrive = new DifferentialDrive(leftMain, rightMain);


    //TODO: Test this, not sure that this value is what we're looking for 
    resetEncoders();
    odometry = new DifferentialDriveOdometry(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // send stuff to smartdashboard

    SmartDashboard.putNumber("Left Position", leftMain.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Position", rightMain.getSelectedSensorPosition());


    SmartDashboard.putNumber("Distance (meters)", (leftMain.getSelectedSensorPosition()*(ENCODER_DISTANCE_METERS_PER_PULSE)));
    SmartDashboard.putNumber("Left Distance (meters)", -1*getMotorPositionsInMeters(leftMain));
    SmartDashboard.putNumber("Right Distance (meters)", getMotorPositionsInMeters(rightMain));
 
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Turn Rate", getTurnRate());
    SmartDashboard.putNumber("Left Velocity (m/s)", -1*leftMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE*10);
    SmartDashboard.putNumber("Right Velocity (m/s)", rightMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE*10);

    SmartDashboard.putNumber("Pigeon Yaw Pitch Roll", getPigeonYawPitchRoll()[0]);

    //TODO: Write Yaw value too dashboard, convert to Double first
    
    //SmartDashboard.putNumber("Rotation 2d (Yaw)", )

  
    odometry.update(getHeading(), -1*getMotorPositionsInMeters(leftMain), getMotorPositionsInMeters(rightMain));
    //resetEncoders(); 
  }

  public void driveWithJoystick(double left, double right) {

    // if not reversed, normal tank drive, else flip left and right and negate the values
    if (isReversed == false) {
      twoMotorDrive.tankDrive(left, right);
    } else {
      twoMotorDrive.tankDrive(-right, -left);
    }

  }

  public void driveWithArcade(double speed, double rotation) {
    twoMotorDrive.arcadeDrive(speed, rotation*-1);
  }

  public void driveForward(double left, double right) {
    twoMotorDrive.tankDrive(left, right);
  }

  public void resetEncoders() {

    rightMain.setSelectedSensorPosition(0);
    leftMain.setSelectedSensorPosition(0);

  }

  // public DifferentialDriveWheelSpeeds getSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(
  //     leftMain.getSelectedSensorVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Constants.WHEEL_RADIUS / 60, // Calculates RPM of the wheel
  //     rightMain.getSelectedSensorVelocity() / Constants.GEAR_RATIO * 2 * Math.PI * Constants.WHEEL_RADIUS / 60 // Calculates RPM of the wheel
  //   );
  // }

  public double getMotorPositionsInMeters(WPI_TalonFX motor) {
    return motor.getSelectedSensorPosition() * ENCODER_DISTANCE_METERS_PER_PULSE;
  }

  // public SimpleMotorFeedforward getFeedForward() {
  //   return feedForward;
  // }

  public Rotation2d getHeading() {

    double[] ypr = {0,0,0};
    pigeon.getYawPitchRoll(ypr);
    //TODO:                                                       V Might be multiplied by negative 1
    return Rotation2d.fromDegrees(Math.IEEEremainder(ypr[0], 360.0));


  }
  
  public void resetHeading() {
    pigeon.setYaw(0.0);
  }

  // public DifferentialDriveKinematics getKinematics() {
  //   return kinematics;
  // }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-1*leftMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE * 10, rightMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE * 10);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMain.setVoltage(-leftVolts);
    rightMain.setVoltage(rightVolts);
    twoMotorDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getMotorPositionsInMeters(leftMain) + (-1 * getMotorPositionsInMeters(rightMain))) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    twoMotorDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    pigeon.setYaw(0);
  }

  public double getTurnRate() {
    double[] xyz_dps = {0,0,0};
    pigeon.getRawGyro(xyz_dps);
    return xyz_dps[0]; // TODO: Test to make sure that we are getting the correct index from the array
  }

  // public void setOutput(double leftVolts, double rightVolts) { 
  //   leftMain.set(leftVolts / 12);
  //   leftMain.set(rightVolts / 12);
  // }

  // public PIDController getLeftPIDController() {
  //    return leftPIDController;
  // }

  // public PIDController getRightPIDController() {
  //   return rightPIDController;
  // }

  public double[] getPigeonYawPitchRoll() { 
    double[] ypr = {0,0,0};
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }
}

