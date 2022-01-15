package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    TalonFX intakeMotor;
    PneumaticsModuleType intakeLeftModuleType = PneumaticsModuleType.REVPH;
    PneumaticsModuleType intakeRightModuleType = PneumaticsModuleType.REVPH;
    private final DoubleSolenoid intakeLeftPiston = new DoubleSolenoid(intakeLeftModuleType, IntakeConstants.INTAKE_LEFT_AIR_IN, IntakeConstants.INTAKE_LEFT_AIR_OUT);
    private final DoubleSolenoid intakeRightPiston = new DoubleSolenoid(intakeRightModuleType, IntakeConstants.INTAKE_RIGHT_AIR_IN, IntakeConstants.INTAKE_RIGHT_AIR_OUT);
    

    public IntakeSubsystem() {
      intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
      intakeMotor.setInverted(false);
    }

    public void runIntake(){
      intakeMotor.set(ControlMode.Velocity, IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void reverseIntake(){
      intakeMotor.set(ControlMode.Velocity, -IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void stopIntake(){
      intakeMotor.set(ControlMode.Velocity, 0);
    }
    
    public void extendIntake() {
      intakeRightPiston.set(Value.kReverse);
      intakeLeftPiston.set(Value.kReverse);
    }
  
    public void retractIntake() {
      intakeRightPiston.set(Value.kForward);
      intakeLeftPiston.set(Value.kForward);
    }

    @Override
    public void periodic() { 
    }
}



