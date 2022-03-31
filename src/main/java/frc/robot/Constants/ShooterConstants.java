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
        Hub, TarmacEdge, BallCircle, LaunchPad, ChuckIt;
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

    public static final NTValue TARMAC_EDGE_UPPER = new NTValue(7250, "Tarmac Edge Upper");
    public static final NTValue TARMAC_EDGE_LOWER = new NTValue(4400, "Tarmac Edge Lower");

    public static final NTValue BALL_CIRCLE_UPPER = new NTValue(8500, "Ball Circle Upper");
    public static final NTValue BALL_CIRCLE_LOWER = new NTValue(4000, "Ball Circle Lower");
    
    public static final NTValue LAUNCH_PAD_UPPER = new NTValue(12000, "Launch Pad Upper"); // 13000
    public static final NTValue LAUNCH_PAD_LOWER = new NTValue(3000, "Launch Pad Lower");  // 2750
   

    // public static final NTValue CHUCK_IT_UPPER = new NTValue(19000, "Chuck it Upper"); 
    // public static final NTValue CHUCK_IT_LOWER = new NTValue(19000, "Chuck it Lower"); 

    public static final double AUTO_INSIDE_TARMAC_LOWER = 5000;
    public static final double AUTO_INSIDE_TARMAC_UPPER = 5500;

    public static final double AUTO_INNER_CIRCLE_TARMAC_LOWER = 5000;
    public static final double AUTO_INNER_CIRCLE_TARMAC_UPPER = 6000;

    public static final double velocityPIDTolerance = 300;

    //SHOOTER DISTANCE - from edge of Hub to Limelight - in Inches
    public static final double FRONT_OF_HUB_DISTANCE = 6;
    public static final double INNER_TARMAC_DISTANCE = 82.935; // 75.5; // 120.8
    public static final double PEAK_TARMAC_DISTANCE = 91.97;
    public static final double BALL_CIRCLE_DISTANCE = 126.31;
    public static final double LAUNCH_PAD_DISTANCE = 176.26; // 128; //245

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
        setShooterSpeedMap(FRONT_OF_HUB_DISTANCE, FRONT_OF_HUB_LOWER.value, FRONT_OF_HUB_UPPER.value);
        //setShooterSpeedMap(INNER_TARMAC_DISTANCE, INNER_TARMAC_LOWER.value, INNER_TARMAC_UPPER.value);
        //setShooterSpeedMap(PEAK_TARMAC_DISTANCE, PEAK_TARMAC_LOWER.value, PEAK_TARMAC_UPPER.value);        
        setShooterSpeedMap(BALL_CIRCLE_DISTANCE, BALL_CIRCLE_LOWER.value, BALL_CIRCLE_UPPER.value);
        setShooterSpeedMap(LAUNCH_PAD_DISTANCE, LAUNCH_PAD_LOWER.value, LAUNCH_PAD_UPPER.value);

        //---------------
        // Add more shot points here.    
        // setShooterSpeedMap(10, 8500.0, 2500.0);  //54.8 
        // setShooterSpeedMap(20, 7250.0, 3000.0);   //    67 
        // setShooterSpeedMap(42, 6000.0, 5000.0);   //78
        // setShooterSpeedMap(60, 5500, 5000);       //96.4
        // setShooterSpeedMap(98, 3900, 8750.0);        //   169 
        // setShooterSpeedMap(112, 3250, 10500.0);   //206.8
    }  
    
} 

