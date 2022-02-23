// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.Constants.DrivetrainConstants;

/** Add your docs here. */
public class AutoTrajectory {

    private static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoDriveConstants.kMaxSpeedMetersPerSecond,
        AutoDriveConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);


    private static List<Translation2d> emptyWaypoints = List.of();

    public static Trajectory GoStraight(double startMeters, double endMeters) {
        
        if (endMeters < 0) {
            trajectoryConfig.setReversed(true);
            endMeters = Math.abs(endMeters);
        }

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startMeters, 0, new Rotation2d(0)),
            emptyWaypoints,
            new Pose2d(endMeters, 0, new Rotation2d(0)),
            trajectoryConfig);
    }

    public static Trajectory RotateInPlace(double startMeters, double degreesToRotate) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startMeters, 0, new Rotation2d(0)),
            emptyWaypoints,
            new Pose2d(startMeters, 0, Rotation2d.fromDegrees(degreesToRotate)),
            trajectoryConfig);
    }


}
