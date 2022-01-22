package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  public TalonFX climberMotor;
  private final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
  private DoubleSolenoid climbPiston;
  double climbSpeed = 0;
    
  public ClimberSubsystem(int motorID, int forwardPistonID, int reversePistonID) {
    climberMotor = new TalonFX(motorID);
    climbPiston = new DoubleSolenoid(moduleType, forwardPistonID, reversePistonID);

    climberMotor.setInverted(false);
  }
  public void extend() {
    climbSpeed = ClimberConstants.CLIMBER_MOTOR_SPEED;
  }

  public void retract() {
    climbSpeed = -ClimberConstants.CLIMBER_MOTOR_SPEED;
  }

  @Override
  public void periodic() { 
    climberMotor.set(ControlMode.PercentOutput, climbSpeed);
    if (climbSpeed == 0) {
      climbPiston.set(Value.kReverse);
    }
    else {
      climbPiston.set(Value.kForward);
    }
    climbSpeed = 0;
  }
}
