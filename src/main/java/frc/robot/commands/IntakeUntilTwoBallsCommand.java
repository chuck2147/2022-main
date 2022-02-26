package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUntilTwoBallsCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;

  public IntakeUntilTwoBallsCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {
    this.intake = intake;
    this.indexer = indexer;
    addRequirements(intake, indexer);
  }

  @Override
  public void execute() {
    intake.runIntakeForward();
  }

  @Override
  public boolean isFinished() {
    return indexer.isLowerTriggered() && indexer.isUpperTriggered();
  }
}
