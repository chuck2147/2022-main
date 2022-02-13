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
    public static int LOWER_MOTOR_ID = 16;
    public static int UPPER_MOTOR_ID = 17;

    //MOTOR PIDs
    public static final double UPPER_P = 0.3;
    public static final double UPPER_I = 0;
    public static final double UPPER_D = 4.5;
    public static final double UPPER_F = 0.0487; 
    public static final double LOWER_P = 0.3;
    public static final double LOWER_I = 0;
    public static final double LOWER_D = 4.5;
    public static final double LOWER_F = 0.0487;

    //MOTORSPEEDs
    public static final NTValue FRONT_OF_HUB_UPPER = new NTValue(-250, "Front of Hub Upper");
    public static final NTValue FRONT_OF_HUB_LOWER = new NTValue(13000, "Front of Hub Lower");

    public static final NTValue BEHIND_TARMAC_UPPER = new NTValue(7500, "Behind Tarmac Upper");
    public static final NTValue BEHIND_TARMAC_LOWER = new NTValue(6500, "Behind Tarmac Lower");
    
    public static final NTValue LAUNCH_PAD_UPPER = new NTValue(18000, "Launch Pad Upper"); 
    public static final NTValue LAUNCH_PAD_LOWER = new NTValue(4000, "Launch Pad Lower"); 
   

    public static final NTValue CHUCK_IT_UPPER = new NTValue(19000, "Chuck it Upper"); 
    public static final NTValue CHUCK_IT_LOWER = new NTValue(19000, "Chuck it Lower"); 

    public static final double velocityPIDTolerance = 100;

    //SHOOTER DISTANCE - from edge of Hub to Limelight - in Inches
    public static final double FRONT_OF_HUB_DISTANCE = 36;
    public static final double BEHIND_TARMAC_DISTANCE = 95;
    public static final double LAUNCH_PAD_DISTANCE = 170;
    public static final double CHUCK_IT_DISTANCE = 0;

    public static final double HUB_HEIGHT = 12 * (8 + (2/3)); // in inches 
    public static final double LIMELIGHT_HEIGHT = 0;
    public static final double LIMELIGHT_ANGLE = 0.588002603548; // in Radians
    
    //SHOOTER STATES 
    public enum ShooterState {
        Hub, Tarmac, LaunchPad, ChuckIt;
    }
} 

