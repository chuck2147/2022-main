// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerStates;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends CommandBase {
  private final IndexerSubsystem indexer;
  private final IndexerConstants.IndexerStates indexerStates;
  /** Creates a new IndexerCommand. */
  public IndexerCommand(IndexerSubsystem indexer, IndexerStates indexerStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.indexerStates = indexerStates;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexerStates == IndexerStates.forward) {
      indexer.runIndexer();
    } else if (indexerStates == IndexerStates.backward){
      indexer.runIndexerReverse();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexerAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
