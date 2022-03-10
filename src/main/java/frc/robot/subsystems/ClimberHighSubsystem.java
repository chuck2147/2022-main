package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.ClimberConstants;

public class ClimberHighSubsystem extends ClimberSubsystem {
  public TalonFX climberMotor;

  public ClimberHighSubsystem() {
    super(ClimberConstants.LOW_CLIMBER_MOTOR_ID, ClimberConstants.CLIMBER_LOW_AIR_IN, ClimberConstants.CLIMBER_LOW_AIR_OUT);
      
  }  
  
}