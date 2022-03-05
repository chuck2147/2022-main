package frc.robot.subsystems;

import java.util.function.Function;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexerSubsystem extends SubsystemBase {
  private final IntakeStateSupplier intakeStateSupplier;
  private AnalogInput irLower = new AnalogInput(IndexerConstants.LOWER_IR_ID);
  private AnalogInput irUpper = new AnalogInput(IndexerConstants.UPPER_IR_ID);
  private final TalonFX lowerMotor = new TalonFX(IndexerConstants.LOWER_MOTOR_ID);
  private final TalonFX upperMotor = new TalonFX(IndexerConstants.UPPER_MOTOR_ID);
  private boolean isFeedingToShooter = false;

  public interface IntakeStateSupplier {
    IntakeStates getState();
  }

  public IndexerSubsystem(IntakeStateSupplier intakeStateSupplier) {
    lowerMotor.setNeutralMode(NeutralMode.Brake);
    upperMotor.setNeutralMode(NeutralMode.Brake);
    this.intakeStateSupplier = intakeStateSupplier;
  }

  public void manualIndexerReverse() {
    // TODO
  }

  public void manualIndexerForward() {
    // TODO
  }

  public boolean isLowerTriggered() {
    return irLower.getVoltage() > IndexerConstants.INDEXER_IR_VOLTAGE;
  }

  public boolean isUpperTriggered() {
    return irUpper.getVoltage() > IndexerConstants.UPPER_IR_VOLTAGE;
  }

  public boolean isFull() {
    return isLowerTriggered() && isUpperTriggered();
  }

  public boolean isEmpty() {
    return !isLowerTriggered() && !isUpperTriggered();
  }

  public void feedToShooter() {
    isFeedingToShooter = true;
  }

  @Override
  public void periodic() {
    double lowerSpeed = 0;
    double upperSpeed = 0;
    //System.out.println(intakeStateSupplier.getState());
    
    if (isFeedingToShooter) { //shooter is up to speed pass the balls in
      upperSpeed = IndexerConstants.UPPER_MOTOR_SPEED_SHOOTING;
      lowerSpeed = IndexerConstants.LOWER_MOTOR_SPEED;
    } 
    else if (intakeStateSupplier.getState() == IntakeStates.Reverse) { //if intake is running in reverse we want indexer to also run in reverse
      upperSpeed = -IndexerConstants.UPPER_MOTOR_SPEED_PLACING;
      lowerSpeed = -IndexerConstants.LOWER_MOTOR_SPEED;
    } 
    else if (isLowerTriggered() && !isUpperTriggered()) { //move ball into top position
      upperSpeed = IndexerConstants.UPPER_MOTOR_SPEED_PLACING;
      lowerSpeed = IndexerConstants.LOWER_MOTOR_SPEED;
    } 
    else if (intakeStateSupplier.getState() == IntakeStates.Forward) {
      if (!isLowerTriggered() && isUpperTriggered()) { //if ball is in top position but not in lower run lower indexer to grab another ball
        upperSpeed = 0;
        lowerSpeed = IndexerConstants.LOWER_MOTOR_SPEED;
      } 
      else if (!isLowerTriggered() && !isUpperTriggered()) { //if no balls are in either position run both indexer motors
        upperSpeed = IndexerConstants.UPPER_MOTOR_SPEED_PLACING;
        lowerSpeed = IndexerConstants.LOWER_MOTOR_SPEED;
      } 
      else { //if both sensors already detect a ball
        upperSpeed = 0;
        lowerSpeed = 0;
      }
    } 
    else { //Otherwise do not run indexer
      upperSpeed = 0;
      lowerSpeed = 0;
    }

    lowerMotor.set(ControlMode.PercentOutput, lowerSpeed);
    upperMotor.set(ControlMode.PercentOutput, upperSpeed);
    isFeedingToShooter = false;
  }
}
