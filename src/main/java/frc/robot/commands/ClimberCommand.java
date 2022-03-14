package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  /** Creates a new ClimberCommand. */
  private final ClimberSubsystem climber;
  private final DoubleSupplier climbSpeedSupplier;

  public ClimberCommand(ClimberSubsystem subsystem, DoubleSupplier speedSupplier) {
    climber = subsystem;
    climbSpeedSupplier = speedSupplier;
    addRequirements(climber);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var speed = climbSpeedSupplier.getAsDouble();
    
    // Use "climber.getType()" instead to figure what climber it is.
    
    //---------------------------------------------
    // NOTE: Put this back in before next practice.
    // if(climber.getEncoderValue() <= 0 && climbSpeedSupplier.getAsDouble() <= 0) {
    //   speed = 0; 
    // }

    climber.runClimber(speed);
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
