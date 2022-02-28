// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.util.StopWatch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.vision.VisionShooting;

public class AutoShootCommand extends CommandBase {
  
  private DrivetrainSubsystem drivetrain;
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer;

  private VisionShooting visionShooting;

  private StopWatch stopWatchShooting;
  private StopWatch stopWatchFallBack;

  private double waitForShootingToBeDoneInSeconds = 3;
  private double waitForFallbackInSeconds = 3;

  private boolean fallbackInitiated = false;
  
  public AutoShootCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    addRequirements(visionSubsystem, shooterSubsystem, indexerSubsystem);
    drivetrain = drivetrainSubsystem;
    shooter = shooterSubsystem;
    indexer = indexerSubsystem;

    visionShooting = new VisionShooting(visionSubsystem);
    stopWatchShooting = new StopWatch();
    stopWatchFallBack = new StopWatch();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopWatchShooting.reset();    
    stopWatchFallBack.reset();

    stopWatchFallBack.start();
    fallbackInitiated = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionShooting.Align(drivetrain);

    if (!fallbackInitiated) {
      // If actively shooting the ball, then start timer.
      if (visionShooting.ShootByDistance(shooter, indexer)) {
        stopWatchShooting.start();
      }
    }
    else {
      // Run fallback shooting.
      // NOTE: Probably can make this a parameter of distance or just speeds that can vary where we think it should be at.
      shooter.setSpeeds(ShooterConstants.BEHIND_TARMAC_LOWER.value, ShooterConstants.BEHIND_TARMAC_UPPER.value);

      if (shooter.isUpToSpeed()) {
        indexer.feedToShooter();
        stopWatchShooting.start();
      }
    }

    // Fallback if shooting never starts after a time.
    if (!stopWatchShooting.isRunning() && stopWatchFallBack.getDuration() > waitForFallbackInSeconds) {
      fallbackInitiated = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when think shooting time should be over.
    return indexer.isEmpty() && (stopWatchShooting.getDuration() > waitForShootingToBeDoneInSeconds);
  }

}
