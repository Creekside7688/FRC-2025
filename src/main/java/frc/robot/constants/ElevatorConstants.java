// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int MOTOR_ID = 10;
    public static final double AUTO_SPEED_UP = 0.8;
	public static final double AUTO_SPEED_DOWN= -0.3;
    public static final double MANUAL_SPEED_UP = 0.8;
	public static final double MANUAL_SPEED_DOWN = -0.3;
	public static final int SENSOR_CHANNEL = 8;
	public static final double STALL_VOLTAGE = 0.6;

    // UNITS: ENCODER ROTATIONS
	public static final double GROUND_HEIGHT = 0;
	public static final double TROUGH_HEIGHT = 1.2;
	public static final double LEVEL_2_HEIGHT = 3.9;
	public static final double LEVEL_3_HEIGHT = 8.0;
	public static final double LEVEL_4_HEIGHT = 0;

	public static final double LEVEL_2_ALGAE_HEIGHT = 1.3;
	public static final double LEVEL_3_ALGAE_HEIGHT = 5.3;

	public static final int LEVEL_0_BUTTON = 0;
	public static final int LEVEL_1_BUTTON = 0;
	public static final int LEVEL_2_BUTTON = 0;
	public static final int LEVEL_3_BUTTON = 0;

	public static final double GEARBOX_RATIO = 1.0/12.0;
}
