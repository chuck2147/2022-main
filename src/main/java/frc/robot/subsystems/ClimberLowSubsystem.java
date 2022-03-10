package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberType;

public class ClimberLowSubsystem extends ClimberSubsystem {
  public TalonFX climberMotor;
  private final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
  private DoubleSolenoid hookPiston;

  public ClimberLowSubsystem() {
    super(ClimberType.Low, ClimberConstants.LOW_CLIMBER_MOTOR_ID, ClimberConstants.CLIMBER_LOW_AIR_IN, ClimberConstants.CLIMBER_LOW_AIR_OUT, ClimberConstants.LOW_CLIMBER_ENCODER_TOP);
    
    hookPiston = new DoubleSolenoid(moduleType, ClimberConstants.HOOK_LOW_AIR_IN, ClimberConstants.HOOK_LOW_AIR_OUT);  
  }
  
  public void pushHook(){
    hookPiston.set(Value.kReverse);
  }

  public void pullHook(){
    hookPiston.set(Value.kForward);
  }
  
}