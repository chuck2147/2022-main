// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import frc.robot.NTValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ShooterConstants{
    //MOTOR ID
    public static int LOWER_SHOOTER_MOTOR_ID = 15;
    public static int UPPER_SHOOTER_MOTOR_ID = 16;
    public static int HOOD_SHOOTER_MOTOR_ID = 17;

    //MOTOR PIDs
    public static final double UPPER_SHOOTER_P = 0.3;
    public static final double UPPER_SHOOTER_I = 0;
    public static final double UPPER_SHOOTER_D = 4.5;
    public static final double UPPER_SHOOTER_F = 0.0487; 
    public static final double LOWER_SHOOTER_P = 0.3;
    public static final double LOWER_SHOOTER_I = 0;
    public static final double LOWER_SHOOTER_D = 4.5;
    public static final double LOWER_SHOOTER_F = 0.0487;

    public static double HOOD_SHOOTER_MOTOR_P = 0;
    public static double HOOD_SHOOTER_MOTOR_I = 0;
    public static double HOOD_SHOOTER_MOTOR_D = 0;
    public static double HOOD_SHOOTER_MOTOR_F = 0;

    //MOTORSPEEDs
    public static double HOOD_SHOOTER_SPEED = 0.25;
    public static final NTValue SHOOTER_TRIANGLE_UPPER = new NTValue(-3000, "Shooter Triangle Upper");
    public static final NTValue SHOOTER_TRIANGLE_LOWER = new NTValue(15000, "Shooter Triangle Lower");

    public static final NTValue SHOOTER_BEHIND_LINE_UPPER = new NTValue(4500, "Shooter Behind Line Upper");
    public static final NTValue SHOOTER_BEHING_LINE_LOWER = new NTValue(10000, "Shooter Behind Line Lower");
    
    public static final NTValue SHOOTER_FRONT_OF_TRENCH_UPPER = new NTValue(4000, "Shooter Front Of Trench Upper"); 
    public static final NTValue SHOOTER_FRONT_OF_TRENCH_LOWER = new NTValue(12750, "Shooter Front OF Trench Lower"); 
   

    public static final NTValue SHOOTER_FAR_UPPER = new NTValue(10000, "Shooter Far Upper"); 
    public static final NTValue SHOOTER_FAR_LOWER = new NTValue(11000, "Shooter Far Lower"); 

    public static final double velocityPIDTolerance = 1000;
} 

