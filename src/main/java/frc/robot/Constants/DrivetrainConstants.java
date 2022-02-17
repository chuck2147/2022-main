// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DrivetrainConstants{
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .5429; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .5524;

    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(167); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-179); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-179);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(135); 

    //LIMELIGHT PIDs 
    public static final double VISION_ALIGN_P = 0.185;
    public static final double VISION_ALIGN_I = 0;
    public static final double VISION_ALIGN_D = 0.005;
    public static final double VISION_ALIGN_F = 0;

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 0;
        public static final double kPYController = 0;
        public static final double kPThetaController = 0;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
} 

