// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Routines;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants.BallCount;
import frc.robot.commands.Autonomous.AutoPathPlanCommand;
import frc.robot.commands.Autonomous.AutoShootCommand;
import frc.robot.commands.Autonomous.ResetOdometryCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ShootAndTaxiPathCommand extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public ShootAndTaxiPathCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
    addRequirements(drivetrain, visionSubsystem, shooter, intake, indexer);

    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("Taxi", AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared, true);
    
    var lowerSpeed = ShooterConstants.AUTO_INSIDE_TARMAC_LOWER;
    var upperSpeed = ShooterConstants.AUTO_INSIDE_TARMAC_LOWER;

    addCommands(
      new ResetOdometryCommand(drivetrain, pathTrajectory.getInitialPose()),
      new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.One, lowerSpeed, upperSpeed).withTimeout(5),
      AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory)
    );
  }
}
