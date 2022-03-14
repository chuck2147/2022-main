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

  private PneumaticsModuleType intakeLeftModuleType = PneumaticsModuleType.REVPH;
  private PneumaticsModuleType intakeRightModuleType = PneumaticsModuleType.REVPH;
  private final DoubleSolenoid intakeLeftPiston = new DoubleSolenoid(intakeLeftModuleType,
      IntakeConstants.INTAKE_LEFT_AIR_IN, IntakeConstants.INTAKE_LEFT_AIR_OUT);
  private final DoubleSolenoid intakeRightPiston = new DoubleSolenoid(intakeRightModuleType,
      IntakeConstants.INTAKE_RIGHT_AIR_IN, IntakeConstants.INTAKE_RIGHT_AIR_OUT);

  public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.setInverted(false);

    stopIntake();
  }

  public void runIntakeForward() {
    intakeState = IntakeStates.Forward;

    setIntake(-IntakeConstants.INTAKE_MOTOR_SPEED, Value.kForward);
  }

  public void runIntakeReverse() {
    intakeState = IntakeStates.Reverse;

    setIntake(IntakeConstants.INTAKE_MOTOR_SPEED, Value.kForward);
  }

  public void stopIntake() {
    intakeState = IntakeStates.Stopped;

    setIntake(0, Value.kReverse);
  }

  public IntakeStates getState() {
    return intakeState;
  }

  @Override
  public void periodic() {
  }

  private void setIntake(double motorSpeed, Value direction) {
    intakeRightPiston.set(direction);
    intakeLeftPiston.set(direction);

    intakeMotor.set(ControlMode.PercentOutput, motorSpeed);
  }
}
