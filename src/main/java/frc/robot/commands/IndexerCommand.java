// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerStates;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexerCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final IndexerConstants.IndexerStates indexerStates;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  /** Creates a new IndexerCommand. */
  public IndexerCommand(IndexerSubsystem indexer, IndexerStates indexerStates, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.indexerStates = indexerStates;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexerStates == IndexerStates.forward) {
      indexer.manualIndexerForward();
    } else if (indexerStates == IndexerStates.backward){
      indexer.manualIndexerReverse();
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /*
When shooter is up to speed, both indexer motors feeds balls forward into the shooter.
  else if lower sensor detects ball & upper sensor doesn't detect a ball, run both indexer motors upto upper sensor
  else if intake is running & upper sensor detects a ball and the lower doesn't detect a ball, run lower index motor forwards only
  else if intake is not running & upper sensor detects & ball and the lower doesn't detect a ball, don't run indexer
  else if intake is running & neither sensor detects a ball, run both index motor forwards
  else if intake is not running & neither sensor detects a ball, don't run indexer
  else if intake is running reversed, run both index motors backwards
  else if both sensors detect a ball, stop index motors




  
*/
@Override
public void periodic() {
    if isUpToSpeed() {
      "run both indexer motors forward"
    }
    if else isLowerTriggered() "but upper sensor isn't triggered"{
      "run both indexer motors until upper sensor is triggered"
    }
    if else "intake is running and" isUpperTriggered() "but not lower sensor" {
      "run lower indexer motors only"
    }
    if else "intake is not running and" isUpperTriggered() "but not lower sensor" {
      "stop both indexer motors"
    }
    if else "intake is running and neither sensor detects a ball" {
      "run both indexer motors forward"
    }
    if else "intake is not running and neither sensor detects a ball" {
      "stop both indexer motors"
    }
    if else "intake is running backwards"{
      "run both indexer motors backwards"
    }
    else {
      "stop both indexer motors"
    }
  }
}
