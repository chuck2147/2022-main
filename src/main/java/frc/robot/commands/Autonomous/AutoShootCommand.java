// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.util.StopWatch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants.BallCount;
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

  private double waitForShootingToBeDoneInSeconds_TwoBall = 1.4;
  private double waitForShootingToBeDoneInSeconds_OneBall = 0.9;
  private double waitForShootingToBeDoneInSeconds;

  private double waitForFallbackInSeconds = 2;

  private boolean fallbackInitiated = false;

  double lowerShooterSpeedDefault;
  double upperShooterSpeedDefault;
  
  public AutoShootCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
                          BallCount ballCountToShoot, double lowerShooterSpeedDefault, double upperShooterSpeedDefault, boolean fallbackShooting) {
    addRequirements(drivetrainSubsystem, visionSubsystem, shooterSubsystem, indexerSubsystem);
    
    waitForShootingToBeDoneInSeconds = (ballCountToShoot == BallCount.One) ? waitForShootingToBeDoneInSeconds_OneBall : waitForShootingToBeDoneInSeconds_TwoBall;

    drivetrain = drivetrainSubsystem;
    shooter = shooterSubsystem;
    indexer = indexerSubsystem;

    this.lowerShooterSpeedDefault = lowerShooterSpeedDefault;
    this.upperShooterSpeedDefault = upperShooterSpeedDefault;

    visionShooting = new VisionShooting(visionSubsystem);
    stopWatchShooting = new StopWatch();
    stopWatchFallBack = new StopWatch();

    fallbackInitiated = fallbackShooting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopWatchShooting.reset();    
    stopWatchFallBack.reset();

    stopWatchFallBack.start();    
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
      shooter.setSpeeds(lowerShooterSpeedDefault, upperShooterSpeedDefault);

      if (visionShooting.IsOnTarget() && shooter.isUpToSpeed()) {
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
