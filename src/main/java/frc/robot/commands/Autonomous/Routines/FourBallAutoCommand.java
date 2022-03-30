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


public class FourBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public FourBallAutoCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
    addRequirements(drivetrain, visionSubsystem, shooter, intake, indexer);

    String pathName1 = "4 Ball Middle Part 1";
    String pathName2 = "4 Ball Middle Part 2";
    String pathName3 = "4 Ball Middle Part 3";

    PathPlannerTrajectory pathTrajectory1 = PathPlanner.loadPath(pathName1, AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);
    PathPlannerTrajectory pathTrajectory2 = PathPlanner.loadPath(pathName2, AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);
    PathPlannerTrajectory pathTrajectory3 = PathPlanner.loadPath(pathName3, AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);

    var lowerSpeed = ShooterConstants.INNER_TARMAC_LOWER.value; // 5000
    var upperSpeed = ShooterConstants.INNER_TARMAC_UPPER.value; // 6000

    addCommands(
      new ResetOdometryCommand(drivetrain, pathTrajectory1.getInitialPose()),
      
      // Get first 2 balls.
      new AutoPathIntakeCommand(intake, IntakeStates.Forward),
      new WaitCommand(1),
      AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory1),
      new AutoPathIntakeCommand(intake, IntakeStates.Stopped),

      // Shoot first 2 balls.
      new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.Two, lowerSpeed, upperSpeed, true).withTimeout(2.5),

      //Get next 2 balls at terminal.
      new AutoPathIntakeCommand(intake, IntakeStates.Forward),
      AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory2),
      new WaitCommand(AutoPathConstants.WAIT_FOR_BALL_ROLL_FROM_TERMINAL),

      // Go back to Tarmac and shoot.
      AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory3),
      new AutoPathIntakeCommand(intake, IntakeStates.Stopped),

      // Shoot last two balls.
      new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.Two, lowerSpeed, upperSpeed, true).withTimeout(2.5)
    );

  }
}
