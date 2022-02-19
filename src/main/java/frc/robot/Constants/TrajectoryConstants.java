// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class TrajectoryConstants {

    private static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoDriveConstants.kMaxSpeedMetersPerSecond,
                AutoDriveConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);


    public static final Trajectory GO_BACKWARDS_TRAJECTORY = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
                new Translation2d(-2, 0)
        ),
        new Pose2d(-2, 0, new Rotation2d(Math.PI/2)),
        trajectoryConfig);


        
        public static final Trajectory TWO_BALL_AUTO_1_TRAJECTORY = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0)
                ),
                new Pose2d(-0.75, 0, new Rotation2d(0)),
                trajectoryConfig);

        public static final Trajectory TWO_BALL_AUTO_2_TRAJECTORY = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0, 0)
                ),
                new Pose2d(1, 0, new Rotation2d(Math.PI)),
                trajectoryConfig);

}

