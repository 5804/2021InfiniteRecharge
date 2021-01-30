/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class VisionSubsystem extends SubsystemBase {

  public double tv;
  public double tx;
  public double ty;
  public double ta;
  public boolean limelightHasValidTarget = false;
  public double limelightDriveCommand = 0.0;
  public double limelightSteerCommand = 0.0;

  /**
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  @Override
  public void periodic() {
    if (!isTargetValid()) {
      limelightHasValidTarget = false;
      limelightDriveCommand = 0.0;
      limelightSteerCommand = 0.0;
    }

    limelightHasValidTarget = true;

    // This is where the arcade drive comes in
  }

  public double getTX() {
    return tx;
  }

  public double getTA() {
    return ta;
  }

  public boolean isTargetValid() {
    return tv == 1.0;
  }
}
