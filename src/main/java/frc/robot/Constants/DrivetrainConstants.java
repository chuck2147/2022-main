// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DrivetrainConstants{
    //PIGEON ID
    public static final int DRIVETRAIN_PIGEON_ID = 0;
    //MOTOR ID
    public static final int DRIVETRAIN_LEFT_MOTOR_ID = 1;
    public static final int DRIVETRAIN_RIGHT_MOTOR_ID = 2;
    public static final double DRIVE_SPEED_SCALE = 0.8;
    //LIMELIGHT PIDs 
    public static final double VISION_ALIGN_P = 0.185;
    public static final double VISION_ALIGN_I = 0;
    public static final double VISION_ALIGN_D = 0.005;
    public static final double VISION_ALIGN_F = 0;
} 

