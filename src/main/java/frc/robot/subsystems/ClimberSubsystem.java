package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  public TalonFX climberMotor;
  private final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
  private DoubleSolenoid climbPiston;
    
  public ClimberSubsystem(int motorID, int forwardPistonID, int reversePistonID) {
    climberMotor = new TalonFX(motorID);
    climbPiston = new DoubleSolenoid(moduleType, forwardPistonID, reversePistonID);

    climberMotor.setInverted(false);
  }

  public void runClimber(double climbSpeed) {
    climberMotor.set(ControlMode.PercentOutput, climbSpeed);

    if (climbSpeed == 0) {
      climbPiston.set(Value.kReverse);
    }
    else {
      climbPiston.set(Value.kForward);
    }
  }
  public double getClimberEncoderValue(){
    return climberMotor.getSelectedSensorPosition();
  }
  
  @Override
  public void periodic() { 
    // double climbSpeed = climbSpeedSupplier.getAsDouble();
    // climberMotor.set(ControlMode.PercentOutput, climbSpeed);
    // if (climbSpeed == 0) {
    //   climbPiston.set(Value.kReverse);
    // }
    // else {
    //   climbPiston.set(Value.kForward);
    // }
  }
}
