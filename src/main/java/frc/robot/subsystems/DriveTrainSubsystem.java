/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrainSubsystem extends SubsystemBase {
  
  public WPI_TalonFX rightMain;
  public WPI_TalonFX rightFollow;
  public WPI_TalonFX leftMain;
  public WPI_TalonFX leftFollow;

  public DifferentialDrive twoMotorDrive;
  
  /**
   * Creates a new DriveTrainSubsystem.
   */
  public DriveTrainSubsystem() {
    leftMain = new WPI_TalonFX(1);
    leftFollow = new WPI_TalonFX(2);
    rightMain = new WPI_TalonFX(3);
    rightFollow = new WPI_TalonFX(4);

    leftFollow.follow(leftMain);
    rightFollow.follow(rightMain);

    twoMotorDrive = new DifferentialDrive(leftMain, rightMain);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWithJoystick(double left, double right){
    twoMotorDrive.tankDrive(left, right);
  }

}
