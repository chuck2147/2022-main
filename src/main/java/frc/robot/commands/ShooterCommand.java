// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final ShooterConstants.ShooterState shooterState;
  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooter, ShooterState shooterState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shooterState = shooterState;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterState == ShooterState.Hub) {
      shooter.shootFromFrontOfHub();
    } else if (shooterState == ShooterState.Tarmac){
      shooter.shootFromBehindTarmac();
    } else if (shooterState == ShooterState.LaunchPad){
      shooter.shootFromLaunchPad();
    } else if (shooterState == ShooterState.ChuckIt) {
      shooter.shootChuckIt();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
