// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import frc.robot.NTValue;
import frc.robot.util.interpolableMap.InterpolatingDouble;
import frc.robot.util.interpolableMap.InterpolatingTreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ShooterConstants{
    //SHOOTER STATES 
    public enum ShooterState {
        Hub, Tarmac, LaunchPad, ChuckIt;
    }

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
    public static final NTValue FRONT_OF_HUB_UPPER = new NTValue(2000, "Front of Hub Upper");
    public static final NTValue FRONT_OF_HUB_LOWER = new NTValue(9000, "Front of Hub Lower");

    public static final NTValue BEHIND_TARMAC_UPPER = new NTValue(6000, "Behind Tarmac Upper");
    public static final NTValue BEHIND_TARMAC_LOWER = new NTValue(4600, "Behind Tarmac Lower");
    
    public static final NTValue LAUNCH_PAD_UPPER = new NTValue(12000, "Launch Pad Upper"); // 13000
    public static final NTValue LAUNCH_PAD_LOWER = new NTValue(3000, "Launch Pad Lower");  // 2750
   

    public static final NTValue CHUCK_IT_UPPER = new NTValue(19000, "Chuck it Upper"); 
    public static final NTValue CHUCK_IT_LOWER = new NTValue(19000, "Chuck it Lower"); 

    public static final double velocityPIDTolerance = 100;

    //SHOOTER DISTANCE - from edge of Hub to Limelight - in Inches
    public static final double FRONT_OF_HUB_DISTANCE = 48;
    public static final double BEHIND_TARMAC_DISTANCE = 120.8;
    public static final double LAUNCH_PAD_DISTANCE = 245;
    public static final double CHUCK_IT_DISTANCE = 0;

    // Setup the map of distances to shooter speeds.
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> UPPER_SHOOTER_SPEED_MAP = new InterpolatingTreeMap<>();
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> LOWER_SHOOTER_SPEED_MAP = new InterpolatingTreeMap<>();
 
    private static void setShooterSpeedMap(double distance, double lowerSpeed, double upperSpeed) {
        var pointDistance = new InterpolatingDouble(distance);

        LOWER_SHOOTER_SPEED_MAP.put(pointDistance, new InterpolatingDouble(lowerSpeed));
        UPPER_SHOOTER_SPEED_MAP.put(pointDistance, new InterpolatingDouble(upperSpeed));
    }  

    // Add to shooter speed map known distances.
    static {
        //----------
        // Add in standard shots.
       //setShooterSpeedMap(FRONT_OF_HUB_DISTANCE, FRONT_OF_HUB_LOWER.value, FRONT_OF_HUB_UPPER.value);
        setShooterSpeedMap(BEHIND_TARMAC_DISTANCE, BEHIND_TARMAC_LOWER.value, BEHIND_TARMAC_UPPER.value);
        setShooterSpeedMap(LAUNCH_PAD_DISTANCE, LAUNCH_PAD_LOWER.value, LAUNCH_PAD_UPPER.value);

        //---------------
        // Add more shot points here.    
        setShooterSpeedMap(54.8, 8500.0, 2500.0);   
        setShooterSpeedMap(67, 7250.0, 3000.0);        
        setShooterSpeedMap(78, 6000.0, 5000.0);   
        setShooterSpeedMap(96.4, 5500, 5000);       
        setShooterSpeedMap(169, 3900, 8750.0);            
        setShooterSpeedMap(206.8, 3250, 10500.0);
    }  
    
} 

