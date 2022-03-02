package frc.robot.Constants;

public class VisionConstants {
    //LIMELIGHT PIDs 
    public static final double VISION_ALIGN_P = 0.5; //0.185
    public static final double VISION_ALIGN_I = 0;
    public static final double VISION_ALIGN_D = 0.005;
    public static final double VISION_ALIGN_F = 0;

    // LIMELIGHT MEASUREMENTS
    public static final double HUB_HEIGHT = 12 * (8 + (2/3)); // in inches 
    public static final double LIMELIGHT_HEIGHT = 32;
    public static final double LIMELIGHT_ANGLE = 0.557;  //0.588002603548; // in Radians

    // TOLERANCES
    public static final double HORIZONTAL_TOLERANCE_IN_DEGREES = 2;
    public static final double DISTANCE_FROM_TARGET_TOLERANCE_IN_INCHES = 3.0;

    // SPEEDS
    public static final double FIND_TARGET_SPEED_MULTIPLIER = 0.8;

    
    //LIMELIGHT IP: 169.254.66.52
}
