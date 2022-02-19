// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CollectCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final VisionSubsystem visionSubsystem;


  private enum CollectState {
    Stopped, WaitingForBalls, WaitingForOneBall, FeedToShooter
  }

  private CollectState collectState = CollectState.Stopped;

  public CollectCommand(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter, VisionSubsystem vision) {
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    this.visionSubsystem = vision;
    // Only using shooter to get values of motors, not setting anything to it.
    // So we aren't passing it to addRequirements()
    addRequirements(intake, indexer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collectState == CollectState.Stopped) {
      if (intake.getRunIntake()) {
        collectState = CollectState.WaitingForBalls;
      }
    }
    
    if (collectState == CollectState.WaitingForBalls) {
      if (indexer.isUpperTriggered()) {
        collectState = CollectState.WaitingForOneBall;
      }
    }

    if (collectState == CollectState.WaitingForOneBall) {
      if (indexer.isLowerTriggered()) {
        collectState = CollectState.Stopped;
      }
    }

    if (collectState == CollectState.FeedToShooter) {
      if (!ReadyToShoot()) {
        // Wait until we get up to speed
        if (indexer.isUpperTriggered()) {
          collectState = CollectState.Stopped;
        } else {
          collectState = CollectState.WaitingForBalls;
        }

      }
    }

    if (!intake.getRunIntake()) {
      collectState = CollectState.Stopped;
    }
    // seperate because we can get to this via any state above
    if (ReadyToShoot()) {
      collectState = CollectState.FeedToShooter;
    }

    switch (collectState) {
      case Stopped:
        intake.retractIntake();
        // All the wheels will stop automatically
        break;
      case WaitingForBalls:
        intake.extendIntake();
        intake.runIntakeForward();
        indexer.run(IndexerConstants.LOWER_MOTOR_SPEED, IndexerConstants.UPPER_MOTOR_SPEED_PLACING);
        break;
      case WaitingForOneBall:
        intake.extendIntake();
        intake.runIntakeForward();
        indexer.run(IndexerConstants.LOWER_MOTOR_SPEED, 0);
        break;
      case FeedToShooter:
        intake.stopAndRetractIntake();
        indexer.run(IndexerConstants.LOWER_MOTOR_SPEED, IndexerConstants.UPPER_MOTOR_SPEED_SHOOTING);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean ReadyToShoot() {
    return shooter.isUpToSpeed();// && (shooter.OverrideVision() || visionSubsystem.IsOnTarget());
  }
}
