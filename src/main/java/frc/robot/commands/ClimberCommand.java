// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimberCommand extends CommandBase {
  /** Creates a new ClimberCommand. */
  private final ClimberSubsystem climberSubsystem;
  private final DoubleSupplier climbSpeedSupplier;
  private final IntakeSubsystem intakeSubsystem;

  public ClimberCommand(ClimberSubsystem subsystem, IntakeSubsystem intake, DoubleSupplier speedSupplier) {
    climberSubsystem = subsystem;
    climbSpeedSupplier = speedSupplier;
    intakeSubsystem = intake;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var speed = climbSpeedSupplier.getAsDouble();
    if (speed != 0) {
      intakeSubsystem.retractIntake();
    }

    // if (climberSubsystem.getClimberEncoderValue() >= 10000){
    //   speed = 0;
    //   if (climbSpeedSupplier.getAsDouble() < 0) {
    //     speed = climbSpeedSupplier.getAsDouble();
    //   }
    // }
    // if (climberSubsystem.getClimberEncoderValue() <= 0) {
    //   speed = 0;
    //   if (climbSpeedSupplier.getAsDouble() > 0) {
    //     speed = climbSpeedSupplier.getAsDouble();
    //   }
    // }
    System.out.println(climberSubsystem.getClimberEncoderValue());
    climberSubsystem.runClimber(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
