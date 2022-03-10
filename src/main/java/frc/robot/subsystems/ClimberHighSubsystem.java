package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberType;

public class ClimberHighSubsystem extends ClimberSubsystem {
  public TalonFX climberMotor;

  public ClimberHighSubsystem() {
    super(ClimberType.High, ClimberConstants.HIGH_CLIMBER_MOTOR_ID, ClimberConstants.CLIMBER_HIGH_AIR_IN, ClimberConstants.CLIMBER_HIGH_AIR_OUT, ClimberConstants.HIGH_CLIMBER_ENCODER_TOP);
      
  }  
  
}