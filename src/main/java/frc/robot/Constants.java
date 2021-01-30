/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final double OUTER_INTAKE_MOTOR_SPEED = .65;
	public static final double INNER_INTAKE_MOTOR_SPEED = 1;
	public static final double CONVEYOR_MOTOR_SPEED = .6;
	public static final double STOP_MOTOR = 0;

	public static final double TOF_SAMPLE_TIME = 25;

	public static final int kTimeoutMs = 30;
	public static final int kPIDLoopIdx = 0;

	public static final Gains kGains_Velocit = new Gains(0.049, 0.1, 0.0, 0.0);
	public static final double DRIVE_FORWARD_TIME = 3;
	public static final double AUTONOMOUS_SPEED = .5;

	public static final double STEER_K = 0.03;
	public static final double DRIVE_K = 0.26;
	public static final double DESIRED_TARGET_AREA = 13.0;
	public static final double MAX_DRIVE = 0.7;
}
