// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Routines;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.AutoPathConstants.PathType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants.BallCount;
import frc.robot.commands.Autonomous.AutoCollectCommand;
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


public class FiveBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public FiveBallAutoCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, 
                             PathType pathType) {
    addRequirements(drivetrain, visionSubsystem, shooter, intake, indexer);

    String pathName1 = null;
    String pathName2 = null;
    String pathName3 = null;

    if (pathType == PathType.Wall) {
      pathName1 = "5 Ball Wall Part 1";
      pathName2 = "5 Ball Wall Part 2";
      pathName3 = "5 Ball Wall Part 3";
    }
    // else if (pathType == PathType.Middle) { // Middle is not supported yet.
    //   pathName1 = "3 Ball Middle";
    //   pathName2 = "Terminal Wall";
    // }

    if (pathName1 != null && pathName2 != null && pathName3 != null) {
      PathPlannerTrajectory pathTrajectory1 = PathPlanner.loadPath(pathName1, AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);
      PathPlannerTrajectory pathTrajectory2 = PathPlanner.loadPath(pathName2, AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);
      PathPlannerTrajectory pathTrajectory3 = PathPlanner.loadPath(pathName3, AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);

      var lowerSpeedStart = ShooterConstants.AUTO_INSIDE_TARMAC_LOWER;
      var upperSpeedStart = ShooterConstants.AUTO_INSIDE_TARMAC_UPPER;

      var lowerSpeedEnd = ShooterConstants.BEHIND_TARMAC_LOWER.value;
      var upperSpeedEnd = ShooterConstants.BEHIND_TARMAC_UPPER.value;

      addCommands(
        new ResetOdometryCommand(drivetrain, pathTrajectory1.getInitialPose()),
        // Shoot the first ball.
        new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.One, lowerSpeedStart, upperSpeedStart).withTimeout(1.5),
        // Go around the tarmac and get 2 more balls and shoot.
        AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory1)
          .deadlineWith(new AutoCollectCommand(BallCount.Two, indexer, intake)),
        new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.Two, lowerSpeedEnd, upperSpeedEnd).withTimeout(2.5),
        // Get next 2 balls at terminal.  
        AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory2)
          .andThen(new WaitCommand(AutoPathConstants.WAIT_FOR_BALL_ROLL_FROM_TERMINAL))
            .deadlineWith(new AutoCollectCommand(BallCount.Two, indexer, intake)),
        // Go back to Tarmac and shoot.
        AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory3),
        new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.Two, lowerSpeedEnd, upperSpeedEnd).withTimeout(2.5)
      );
    }

  }
}
