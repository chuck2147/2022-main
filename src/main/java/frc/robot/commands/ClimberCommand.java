// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimberCommand extends CommandBase {
  /** Creates a new ClimberCommand. */
  private final ClimberSubsystem climberSubsystem;
  private final DoubleSupplier climbSpeedSupplier;
  private final IntakeSubsystem intakeSubsystem;
  private final double encoderValueTop;
  

  public ClimberCommand(ClimberSubsystem subsystem, IntakeSubsystem intake, DoubleSupplier speedSupplier, double encoderValueTop) {
    climberSubsystem = subsystem;
    climbSpeedSupplier = speedSupplier;
    intakeSubsystem = intake;
    this.encoderValueTop = encoderValueTop;
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

    if (Math.abs(climberSubsystem.getEncoderValue()) >= Math.abs(encoderValueTop)){
      speed = 0;
      if(encoderValueTop == ClimberConstants.RIGHT_CLIMBER_ENCODER_TOP) {
        if (climbSpeedSupplier.getAsDouble() < 0) {
          speed = climbSpeedSupplier.getAsDouble();
        }
        //need to continue testing left side of climber, it not working correctly
      } else if (encoderValueTop == ClimberConstants.LEFT_CLIMBER_ENCODER_TOP) {
        if (climbSpeedSupplier.getAsDouble() < 0) {
          speed = climbSpeedSupplier.getAsDouble();
        }
      }
    }
    if (Math.abs(climberSubsystem.getEncoderValue()) == 0) {
      speed = 0;
      if (encoderValueTop == ClimberConstants.RIGHT_CLIMBER_ENCODER_TOP) {
        if (climbSpeedSupplier.getAsDouble() > 0) {
          speed = climbSpeedSupplier.getAsDouble();
        }
        //need to continue testing left side of climber, it not working correctly
      } else if (encoderValueTop == ClimberConstants.LEFT_CLIMBER_ENCODER_TOP) {
        if (climbSpeedSupplier.getAsDouble() > 0) {
          speed = climbSpeedSupplier.getAsDouble();
        }
      }
    }
    //System.out.println(climbSpeedSupplier.getAsDouble());
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
