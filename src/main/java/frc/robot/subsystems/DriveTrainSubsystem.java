/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
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

  public boolean isReversed = false;
  
  /**
   * Creates a new DriveTrainSubsystem.
   */
  public DriveTrainSubsystem() {
    leftMain = new WPI_TalonFX(1);
    leftFollow = new WPI_TalonFX(2);
    rightMain = new WPI_TalonFX(3);
    rightFollow = new WPI_TalonFX(4);

    pigeon = new PigeonIMU(1);

    leftMain.setInverted(true);
    leftFollow.setInverted(true); 
    rightMain.setInverted(true);
    rightFollow.setInverted(true);

    leftFollow.follow(leftMain);
    rightFollow.follow(rightMain);

    leftMain.setNeutralMode(NeutralMode.Coast);
    rightMain.setNeutralMode(NeutralMode.Coast);
    leftFollow.setNeutralMode(NeutralMode.Coast);
    rightFollow.setNeutralMode(NeutralMode.Coast);

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

    SmartDashboard.putNumber("Left Velocity", leftMain.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity", rightMain.getSelectedSensorVelocity());


    //TODO: Write Yaw value too dashboard, convert to Double first
    
    //SmartDashboard.putNumber("Rotation 2d (Yaw)", )

   // odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters)
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
    twoMotorDrive.arcadeDrive(speed, rotation);
  }

  public void driveForward(double left, double right) {
    twoMotorDrive.tankDrive(left, right);
  }

public void resetEncoders() {

  rightMain.setSelectedSensorPosition(0.0);
  leftMain.setSelectedSensorPosition(0.0);

}

public Rotation2d getHeading() {

  double[] ypr = {0,0,0};
  pigeon.getYawPitchRoll(ypr);
  return Rotation2d.fromDegrees(Math.IEEEremainder(ypr[0], 360.0d) * -1.0d);

}


public void resetHeading() {
  pigeon.setYaw(0.0);
}



}

