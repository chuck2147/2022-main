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
  
  private double lowerTargetSpeed = 0;
  private double upperTargetSpeed = 0;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooter, ShooterState shooterState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shooterState = shooterState;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.SetOverrideVision(true);

    if (shooterState == ShooterState.Hub) {
      shootFromFrontOfHub();
    } else if (shooterState == ShooterState.Tarmac){
      shootFromBehindTarmac();
    } else if (shooterState == ShooterState.LaunchPad){
      shootFromLaunchPad();
    } else if (shooterState == ShooterState.ChuckIt) {
      shootChuckIt();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeeds(lowerTargetSpeed, upperTargetSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.SetOverrideVision(false);
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void shootFromBehindTarmac() {
    lowerTargetSpeed = ShooterConstants.BEHIND_TARMAC_LOWER.value;
    upperTargetSpeed = ShooterConstants.BEHIND_TARMAC_UPPER.value;
  }

  private void shootFromFrontOfHub() {
    lowerTargetSpeed = ShooterConstants.FRONT_OF_HUB_LOWER.value;
    upperTargetSpeed = ShooterConstants.FRONT_OF_HUB_UPPER.value;
  }

  private void shootFromLaunchPad(){
    lowerTargetSpeed = ShooterConstants.LAUNCH_PAD_LOWER.value;
    upperTargetSpeed = ShooterConstants.LAUNCH_PAD_UPPER.value;
  }
  
  private void shootChuckIt() {
    lowerTargetSpeed = ShooterConstants.CHUCK_IT_LOWER.value;
    upperTargetSpeed = ShooterConstants.CHUCK_IT_UPPER.value;
  }
}
