// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoPathIntakeCommand extends CommandBase {
  
  private final IntakeSubsystem intake;
  private boolean done = false;
  private IntakeStates intakeState;

  public AutoPathIntakeCommand(IntakeSubsystem intake, IntakeStates intakeState) {

    this.intake = intake;
    this.intakeState = intakeState;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (intakeState == IntakeStates.Forward) {
      intake.runIntakeForward();
    }
    else if (intakeState == IntakeStates.Stopped) {
      intake.stopIntake();
    }
    
    done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
