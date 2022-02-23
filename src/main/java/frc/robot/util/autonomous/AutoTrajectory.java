// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.autonomous;

import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
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
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startMeters, 0, new Rotation2d(0)),
            List.of(
               // new Translation2d(-1, 0)
            ),
            new Pose2d(endMeters, 0, new Rotation2d(0)),
            trajectoryConfig);

            // ControlVectorList vectorList = new ControlVectorList();
            // vectorList.add(new ControlVector(startMeters, 0.0))

            // return TrajectoryGenerator.generateTrajectory(
            //     List.of(
            //         new ControlVector(startMeters, 0.0),
            //         new ControlVector(startMeters, 0.0)
            //         ),
            //     trajectoryConfig);
    }

    public static Trajectory RotateInPlace(double startMeters, double endMeters, double degreesToRotate) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(startMeters, 0, new Rotation2d(0)),
            emptyWaypoints,
            new Pose2d(startMeters, 0, new Rotation2d(Math.PI)),
            trajectoryConfig);
    }


}
