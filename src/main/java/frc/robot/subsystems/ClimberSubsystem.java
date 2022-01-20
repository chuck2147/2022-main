package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberType;

public class ClimberSubsystem extends SubsystemBase {
  public TalonFX climberMotor;
  PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
  private DoubleSolenoid climbPiston;
    
  public ClimberSubsystem(ClimberType climberType) {
    climberMotor = getMotorFromType(climberType);
    climbPiston = getPistonFromType(climberType);

    climberMotor.setInverted(false);
  }

  private TalonFX getMotorFromType(ClimberType climberType) {
    int motorId = (climberType == ClimberType.Low) ? ClimberConstants.LOW_CLIMBER_MOTOR_ID : ClimberConstants.HIGH_CLIMBER_MOTOR_ID;

    return new TalonFX(motorId);
  }

  private DoubleSolenoid getPistonFromType(ClimberType climberType) {
    int forwardChannel;
    int reverseChannel;

    if (climberType == ClimberType.Low) {
      forwardChannel = ClimberConstants.CLIMBER_LOW_AIR_IN;
      reverseChannel = ClimberConstants.CLIMBER_LOW_AIR_OUT;
    }
    else {      
      forwardChannel = ClimberConstants.CLIMBER_HIGH_AIR_IN;
      reverseChannel = ClimberConstants.CLIMBER_HIGH_AIR_OUT;
    }

    return new DoubleSolenoid(moduleType, forwardChannel, reverseChannel);
  }

  public void extendClimber(ClimberType climberType) {
    climberMotor.set(ControlMode.PercentOutput, 1);
  }

  public void reverseClimber(ClimberType climberType) {
    climberMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stopClimber(ClimberType climberType) {
    climberMotor.set(ControlMode.PercentOutput, 0);
  }

  public void climberPistonOn(ClimberType climberType) {
    climbPiston.set(Value.kReverse);
  }

  public void climberPistonOff(ClimberType climberType) {
    climbPiston.set(Value.kForward);
  }

  @Override
  public void periodic() { 
  }
}
