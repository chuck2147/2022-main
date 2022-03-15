// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants.BallCount;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCollectCommand extends CommandBase {
  private BallCount ballCount;
  
  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;

  public AutoCollectCommand(BallCount ballCountToStopAt, IndexerSubsystem indexer, IntakeSubsystem intake) {
    ballCount = ballCountToStopAt;

    this.intake = intake;
    this.indexer = indexer;
    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntakeForward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = false;

    if (ballCount == BallCount.One) {
      done = indexer.isUpperTriggered();
    }
    else {
      done = indexer.isFull();
    }

    return done;
  }
}
