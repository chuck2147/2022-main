package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final IntakeStates intakeState;

  public IntakeCommand(IntakeSubsystem intake, IntakeStates intakeState) {
    this.intake = intake;
    this.intakeState = intakeState;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeState == IntakeStates.Forward) {
      intake.runIntakeForward();
    }
    else if (intakeState == IntakeStates.Reverse) {
      intake.runIntakeReverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
