package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  /** Creates a new ClimberCommand. */
  private final ClimberSubsystem climber;
  private final DoubleSupplier climbSpeedSupplier;
  private final String climberSubsystemName;

  public ClimberCommand(ClimberSubsystem subsystem, DoubleSupplier speedSupplier) {
    climber = subsystem;
    climbSpeedSupplier = speedSupplier;
    addRequirements(climber);

    climberSubsystemName = subsystem.getClass().getSimpleName();
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var speed = climbSpeedSupplier.getAsDouble();

    // ---------------------------------
    // NOTE:  Keeping the old redundant code here for reference, but commenting it out for now.
    //        Cleaner more performant code is below.

    // if (Math.abs(climber.getEncoderValue()) >= Math.abs(climber.getTopEncoderValue())){
    //   speed = 0;
    //   if(climber.getTopEncoderValue() == ClimberConstants.LOW_CLIMBER_ENCODER_TOP) {
    //     if (climbSpeedSupplier.getAsDouble() < 0) {
    //       speed = climbSpeedSupplier.getAsDouble();
    //     }
    //     //need to continue testing left side of climber, it not working correctly
    //   } else if (climber.getTopEncoderValue() == ClimberConstants.HIGH_CLIMBER_ENCODER_TOP) {
    //     if (climbSpeedSupplier.getAsDouble() < 0) {
    //       speed = climbSpeedSupplier.getAsDouble();
    //     }
    //   }
    // }
    // if (Math.abs(climber.getEncoderValue()) == 0) {
    //   speed = 0;
    //   if (climber.getTopEncoderValue() == ClimberConstants.LOW_CLIMBER_ENCODER_TOP) {
    //     if (climbSpeedSupplier.getAsDouble() > 0) {
    //       speed = climbSpeedSupplier.getAsDouble();
    //     }
    //     //need to continue testing left side of climber, it not working correctly
    //   } else if (climber.getTopEncoderValue() == ClimberConstants.HIGH_CLIMBER_ENCODER_TOP) {
    //     if (climbSpeedSupplier.getAsDouble() > 0) {
    //       speed = climbSpeedSupplier.getAsDouble();
    //     }
    //   }
    // }
    
    // Use "climberSubsystemName" instead to figure what climber it is.
    if (Math.abs(climber.getEncoderValue()) >= Math.abs(climber.getTopEncoderValue())){
      speed = 0;

      if (climbSpeedSupplier.getAsDouble() < 0) {
        speed = climbSpeedSupplier.getAsDouble();
      }
      //need to continue testing left side of climber, it not working correctly
    }

    if (Math.abs(climber.getEncoderValue()) == 0) {
      speed = 0;

      if (climbSpeedSupplier.getAsDouble() > 0) {
        speed = climbSpeedSupplier.getAsDouble();
      }
        //need to continue testing left side of climber, it not working correctly      
    }
    
    //System.out.println(climbSpeedSupplier.getAsDouble());
    climber.runClimber(speed);
    climber.setClimberPiston(speed);
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
