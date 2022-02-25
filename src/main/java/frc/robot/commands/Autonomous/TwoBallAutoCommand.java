// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeUntilTwoBallsCommand;
import frc.robot.commands.ShootByVisionCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.autonomous.AutoTrajectory;

public class TwoBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public TwoBallAutoCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
    addRequirements(drivetrain, visionSubsystem, shooter);

    var endMeters = 2.0;
    var goStraightTrajectory = AutoTrajectory.GoStraight(0, endMeters);
    var rotateTrajectory = AutoTrajectory.RotateInPlace(endMeters, 180);

    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(goStraightTrajectory.getInitialPose())), 
      new ParallelCommandGroup(
        new IntakeUntilTwoBallsCommand(indexer, intake),
        new SequentialCommandGroup(
          AutoDriveBaseCommand.GetCommand(drivetrain, goStraightTrajectory),
          AutoDriveBaseCommand.GetCommand(drivetrain, rotateTrajectory)
        )
      ),
      new ShootByVisionCommand(drivetrain, visionSubsystem, shooter, () -> 0, () -> 0)
    );
  }
}
