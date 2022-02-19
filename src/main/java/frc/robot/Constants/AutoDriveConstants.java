// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class AutoDriveConstants {
    public static final double kMaxSpeedMetersPerSecond = DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;

    public static final double kMaxAngularSpeedRadiansPerSecond = DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 10;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 4;

    public static final double kPXController = 1;
    public static final double kIXController = 0;
    public static final double kDXController = 0;

    public static final double kPYController = 1;
    public static final double kIYController = 0;
    public static final double kDYController = 0;

    public static final double kPThetaController = 3;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}
