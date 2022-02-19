package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  public TalonFX climberMotor;
  private final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
  private DoubleSolenoid climbPiston;
  private double encoderOffset = 0;
  private DigitalInput limitSwitch;
    
  public ClimberSubsystem(int motorID, int forwardPistonID, int reversePistonID, DigitalInput limitSwitch) {
    climberMotor = new TalonFX(motorID);
    climbPiston = new DoubleSolenoid(moduleType, forwardPistonID, reversePistonID);
    this.limitSwitch = limitSwitch;

    climberMotor.setInverted(false);
  }
  
  public ClimberSubsystem() {
    climberMotor.setNeutralMode(NeutralMode.Brake);
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
  public double getEncoderValue(){
    return climberMotor.getSelectedSensorPosition() - encoderOffset;
  }
  
  public void setClimberPiston(double climbSpeed){
    if (Math.abs(climbSpeed) < 0.01) {
      climbPiston.set(Value.kReverse);
    }
    else {
      climbPiston.set(Value.kForward);
    }
  }
  @Override
  public void periodic() { 
    if(!limitSwitch.get()) {
      encoderOffset = climberMotor.getSelectedSensorPosition(); 
    }
    //System.out.println(limitSwitch.get());
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
