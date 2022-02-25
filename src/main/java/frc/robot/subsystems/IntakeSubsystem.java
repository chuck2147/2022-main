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
  TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
  IntakeStates intakeState = IntakeStates.Stopped;

  PneumaticsModuleType intakeLeftModuleType = PneumaticsModuleType.REVPH;
  PneumaticsModuleType intakeRightModuleType = PneumaticsModuleType.REVPH;
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
  }

  public void runIntakeReverse() {
    intakeState = IntakeStates.Reverse;
  }

  public IntakeStates getState() {
    return intakeState;
  }

  @Override
  public void periodic() {
    double motorSpeed = 0;
    if (intakeState == IntakeStates.Forward) {
      motorSpeed = -IntakeConstants.INTAKE_MOTOR_SPEED;
      intakeRightPiston.set(Value.kForward);
      intakeLeftPiston.set(Value.kForward);
    } else if (intakeState == IntakeStates.Reverse) {
      motorSpeed = IntakeConstants.INTAKE_MOTOR_SPEED;
      intakeRightPiston.set(Value.kForward);
      intakeLeftPiston.set(Value.kForward);
    } else {
      intakeRightPiston.set(Value.kReverse);
      intakeLeftPiston.set(Value.kReverse);
    }

    intakeMotor.set(ControlMode.PercentOutput, motorSpeed);

    intakeState = IntakeStates.Stopped;
  }
}
