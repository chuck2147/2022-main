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
public final class IndexerConstants{
    //MOTOR ID
    public static int INDEXER_MOTOR_ID = 13;
    public static int HOPPER_MOTOR_ID = 14;

    //MOTOR SPEED
    public static double INDEXER_MOTOR_SPEED = -.25;
    public static double HOPPER_MOTOR_SPEED = .25;

    //IR ID
    public static int INDEXER_IR_ID = 0;
    public static int HOPPER_IR_ID = 1;

    //IR VOLTAGE LIMITS
    public static double INDEXER_IR_VOLTAGE = 1.75;
    public static double HOPPER_IR_VOLTAGE = 1.2;

} 

