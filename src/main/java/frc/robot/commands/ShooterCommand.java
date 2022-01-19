package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
  ShooterSubsystem shooterSubsystem;
  ShooterState shooterState;
  public ShooterCommand(ShooterSubsystem subsystem, ShooterState shooterState)  {
    this.shooterState = shooterState;
    this.shooterSubsystem = subsystem;
    addRequirements(subsystem);
  }
    @Override
  public void execute() {
    if (shooterState == ShooterState.Tarmac) {
      shooterSubsystem.shootFromBehindTarmac();
    }else if(shooterState == ShooterState.Hub) {
      shooterSubsystem.shootFromFrontOfHub();
    }else if(shooterState == ShooterState.LaunchPad) {
      shooterSubsystem.shootFromLaunchPad();
    }else if(shooterState == ShooterState.ChuckIt) {
      shooterSubsystem.shootChuckIt();
    }
  }
  @Override 
  public void initialize() {

  }
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }
  @Override
  public boolean isFinished() {
    return true;
  }
}
