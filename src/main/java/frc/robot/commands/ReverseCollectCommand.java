// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseCollectCommand extends CommandBase {
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;

    public ReverseCollectCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake, indexer);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intake.runIntakeReverse();
      indexer.manualIndexerReverse();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
