package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem intake;
  private final IntakeConstants.IntakeStates intakeStates;

  public IntakeCommand(IntakeSubsystem intake, IntakeStates intakeStates){
    this.intake = intake;
    this.intakeStates = intakeStates;
    addRequirements(intake);
  }
}
