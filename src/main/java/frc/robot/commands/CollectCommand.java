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

public class CollectCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final IntakeSubsystem intake; 
  private final ShooterSubsystem shooter;
  private final Button extendIntakeButton;
  private enum CollectState {
    Stopped, WaitingForBalls, WaitingForOneBall, FeedToShooter
  }
  private CollectState collectState = CollectState.Stopped;

  public CollectCommand(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter, Button extendIntakeButton) {
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    this.extendIntakeButton = extendIntakeButton;
    //Only using shooter to get values of motors, not setting anything to it.
    //So we aren't passing it to addRequirements()
    addRequirements(intake, indexer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collectState == CollectState.Stopped) {
      if (extendIntakeButton.get()){
        collectState = CollectState.WaitingForBalls;
      }
    } else if (collectState == CollectState.WaitingForBalls) {
      if(indexer.isUpperTriggered()) {
        collectState = CollectState.WaitingForOneBall;
      }
    } else if (collectState == CollectState.WaitingForOneBall) {
      if(indexer.isLowerTriggered()) {
        collectState = CollectState.Stopped;
      }
    } else if (collectState == CollectState.FeedToShooter) {
      if(!shooter.isUpToSpeed()) {
        //Wait until we get up to speed
        collectState = CollectState.Stopped;
      }
    }
    
    
    //seperate because we can get to this via any state above
    if(shooter.isUpToSpeed()) {
      collectState = CollectState.FeedToShooter;
    }

    System.out.println(collectState);


    switch (collectState) {
      case Stopped:
        intake.retractIntake();
        // All the wheels will stop automatically
        break;
      case WaitingForBalls:
        intake.extendIntake();
        intake.runIntakeForward();
        indexer.run(IndexerConstants.LOWER_MOTOR_SPEED, IndexerConstants.UPPER_MOTOR_SPEED);
        break;
      case WaitingForOneBall:
        intake.extendIntake();
        intake.runIntakeForward();
        indexer.run(IndexerConstants.LOWER_MOTOR_SPEED, 0);
        break;
      case FeedToShooter:
        intake.retractIntake();
        indexer.run(IndexerConstants.LOWER_MOTOR_SPEED, IndexerConstants.UPPER_MOTOR_SPEED);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
