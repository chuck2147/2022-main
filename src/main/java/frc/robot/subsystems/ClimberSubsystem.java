package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  public TalonFX leftClimberMotor = new TalonFX(ClimberConstants.LEFT_CLIMBER_MOTOR_ID);
  public TalonFX rightClimberMotor = new TalonFX(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID);
  PneumaticsModuleType climbLeftModuleType = PneumaticsModuleType.REVPH;
  PneumaticsModuleType climbRightModuleType = PneumaticsModuleType.REVPH;
  private final DoubleSolenoid climbRightPiston = new DoubleSolenoid(climbRightModuleType, ClimberConstants.CLIMBER_RIGHT_AIR_IN, ClimberConstants.CLIMBER_RIGHT_AIR_OUT);
  private final DoubleSolenoid climbLeftPiston = new DoubleSolenoid(climbLeftModuleType, ClimberConstants.CLIMBER_LEFT_AIR_IN, ClimberConstants.CLIMBER_LEFT_AIR_OUT);

 

    
  public ClimberSubsystem() {
    leftClimberMotor.setInverted(false);
    rightClimberMotor.setInverted(false);

  }

  public void runClimber(){
    leftClimberMotor.set(ControlMode.Velocity, ClimberConstants.CLIMBER_MOTOR_SPEED);
    rightClimberMotor.set(ControlMode.Velocity, ClimberConstants.CLIMBER_MOTOR_SPEED);
  }

    public void reverseClimber(){
    leftClimberMotor.set(ControlMode.Velocity, ClimberConstants.CLIMBER_MOTOR_SPEED);
    rightClimberMotor.set(ControlMode.Velocity, ClimberConstants.CLIMBER_MOTOR_SPEED);
  }
  public void stopClimber(){
    leftClimberMotor.set(ControlMode.Velocity, 0);
    rightClimberMotor.set(ControlMode.Velocity, 0);
  }
  public void extendClimber() {
    climbRightPiston.set(Value.kReverse);
    climbLeftPiston.set(Value.kReverse);
  }
  public void retractClimber() {
    climbRightPiston.set(Value.kForward);
    climbLeftPiston.set(Value.kForward);
  }
  @Override
  public void periodic() { 
  }
}
