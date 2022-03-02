package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.subsystems.IndexerSubsystem.IntakeStateSupplier;

public class IntakeSubsystem extends SubsystemBase implements IntakeStateSupplier {
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
  private IntakeStates intakeState = IntakeStates.Stopped;
  private boolean wasStateSet = false;

  private PneumaticsModuleType intakeLeftModuleType = PneumaticsModuleType.REVPH;
  private PneumaticsModuleType intakeRightModuleType = PneumaticsModuleType.REVPH;
  private final DoubleSolenoid intakeLeftPiston = new DoubleSolenoid(intakeLeftModuleType,
      IntakeConstants.INTAKE_LEFT_AIR_IN, IntakeConstants.INTAKE_LEFT_AIR_OUT);
  private final DoubleSolenoid intakeRightPiston = new DoubleSolenoid(intakeRightModuleType,
      IntakeConstants.INTAKE_RIGHT_AIR_IN, IntakeConstants.INTAKE_RIGHT_AIR_OUT);

  public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.setInverted(false);
  }

  public void runIntakeForward() {
    intakeState = IntakeStates.Forward;
    wasStateSet = true;
  }

  public void runIntakeReverse() {
    intakeState = IntakeStates.Reverse;
    wasStateSet = true;
  }

  public IntakeStates getState() {
    return intakeState;
  }

  @Override
  public void periodic() {
    if(!wasStateSet){
      intakeState = IntakeStates.Stopped;
    }

    double motorSpeed = 0;
    if (intakeState == IntakeStates.Forward) {
      motorSpeed = -IntakeConstants.INTAKE_MOTOR_SPEED;
      intakeRightPiston.set(Value.kForward);
      intakeLeftPiston.set(Value.kForward);
    } 
    else if (intakeState == IntakeStates.Reverse) {
      motorSpeed = IntakeConstants.INTAKE_MOTOR_SPEED;
      intakeRightPiston.set(Value.kForward);
      intakeLeftPiston.set(Value.kForward);
    } 
    else {
      intakeRightPiston.set(Value.kReverse);
      intakeLeftPiston.set(Value.kReverse);
    }

    intakeMotor.set(ControlMode.PercentOutput, motorSpeed);

    wasStateSet = false;
  }
}
