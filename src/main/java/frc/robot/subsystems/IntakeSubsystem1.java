package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    PneumaticsModuleType intakeLeftModuleType = PneumaticsModuleType.REVPH;
    PneumaticsModuleType intakeRightModuleType = PneumaticsModuleType.REVPH;
    private final DoubleSolenoid intakeLeftPiston = new DoubleSolenoid(intakeLeftModuleType, IntakeConstants.INTAKE_LEFT_AIR_IN, IntakeConstants.INTAKE_LEFT_AIR_OUT);
    private final DoubleSolenoid intakeRightPiston = new DoubleSolenoid(intakeRightModuleType, IntakeConstants.INTAKE_RIGHT_AIR_IN, IntakeConstants.INTAKE_RIGHT_AIR_OUT);
    public IntakeSubsystem() {
      intakeMotor.setInverted(false);
    }
}
