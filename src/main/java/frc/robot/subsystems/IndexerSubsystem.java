package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private AnalogInput irLower = new AnalogInput(IndexerConstants.LOWER_IR_ID);
  private AnalogInput irUpper = new AnalogInput(IndexerConstants.UPPER_IR_ID);
  TalonFX lowerMotor = new TalonFX(IndexerConstants.LOWER_MOTOR_ID);
  TalonFX upperMotor = new TalonFX(IndexerConstants.UPPER_MOTOR_ID);
  double lowerSpeed = 0;
  double upperSpeed = 0;

  public IndexerSubsystem(){
    lowerMotor.setNeutralMode(NeutralMode.Brake);
    upperMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void manualIndexerReverse() {
    lowerSpeed = -IndexerConstants.LOWER_MOTOR_SPEED;
    upperSpeed = -IndexerConstants.UPPER_MOTOR_SPEED;
  }

  public void manualIndexerForward() {
    lowerSpeed = IndexerConstants.LOWER_MOTOR_SPEED;
    upperSpeed = IndexerConstants.UPPER_MOTOR_SPEED;
  }

  public void run(double lower, double upper){
    lowerSpeed = lower;
    upperSpeed = upper;
  }

  public boolean isLowerTriggered() {
    return irLower.getVoltage() > IndexerConstants.INDEXER_IR_VOLTAGE;
  }

  public boolean isUpperTriggered() {
    return irUpper.getVoltage() > IndexerConstants.HOPPER_IR_VOLTAGE;
  }

  @Override
  public void periodic() {
    lowerMotor.set(ControlMode.PercentOutput, lowerSpeed);
    upperMotor.set(ControlMode.PercentOutput, upperSpeed);
    lowerSpeed = 0;
    upperSpeed = 0;
  }
}
