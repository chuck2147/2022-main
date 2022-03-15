// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Routines;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants.BallCount;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.commands.Autonomous.AutoCollectCommand;
import frc.robot.commands.Autonomous.AutoPathPlanCommand;
import frc.robot.commands.Autonomous.AutoShootCommand;
import frc.robot.commands.Autonomous.AutoPathIntakeCommand;
import frc.robot.commands.Autonomous.ResetOdometryCommand;
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
    addRequirements(drivetrain, visionSubsystem, shooter, intake, indexer);

    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("2 Ball", AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);

    var lowerSpeed = ShooterConstants.BEHIND_TARMAC_LOWER.value;
    var upperSpeed = ShooterConstants.BEHIND_TARMAC_UPPER.value;

    addCommands(
      new ResetOdometryCommand(drivetrain, pathTrajectory.getInitialPose()),
      // Get first 2 balls.
      // (new WaitCommand(0.5))
      //   .andThen(AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory))
      //     .deadlineWith(new AutoCollectCommand(BallCount.Two, indexer, intake)),
      // new SequentialCommandGroup(
      //   new WaitCommand(0.5),
      //   AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory1))
      //     .deadlineWith(new AutoCollectCommand(BallCount.Two, indexer, intake)),

      // AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory)
      //     .deadlineWith(new AutoCollectCommand(BallCount.Two, indexer, intake)),

      // Get first 2 balls.
      new AutoPathIntakeCommand(intake, IntakeStates.Forward),
      new WaitCommand(1),
      AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory),
      new AutoPathIntakeCommand(intake, IntakeStates.Stopped),

      // Shoot 2 balls
      new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.Two, lowerSpeed, upperSpeed).withTimeout(3)
    );

  }
}
