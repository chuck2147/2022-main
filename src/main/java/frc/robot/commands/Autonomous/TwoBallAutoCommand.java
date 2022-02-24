// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.commands.CollectCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.ShootByVisionCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class TwoBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public TwoBallAutoCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
    addRequirements(drivetrain, visionSubsystem, shooter);

    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("2 Ball", AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);

    addCommands(
      new ResetOdometryCommand(drivetrain, pathTrajectory.getInitialPose()),
      AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory)
    );

    // addCommands(
    //   new ExtendIntakeCommand(intake),
    //   new InstantCommand(() -> drivetrain.resetOdometry(goStraightTrajectory.getInitialPose())), 
    //   new ParallelCommandGroup(
    //     new CollectCommand(indexer, intake, shooter, visionSubsystem),
    //     new SequentialCommandGroup(
    //       AutoDriveBaseCommand.GetCommand(drivetrain, goStraightTrajectory),
    //       AutoDriveBaseCommand.GetCommand(drivetrain, rotateTrajectory),
    //       new ShootByVisionCommand(drivetrain, visionSubsystem, shooter, () -> 0, () -> 0)
    //     )
    //   )
    // );
  }
}
