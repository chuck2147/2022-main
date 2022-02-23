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
    upperSpeed = -IndexerConstants.UPPER_MOTOR_SPEED_PLACING;
  }

  public void manualIndexerForward() {
    lowerSpeed = IndexerConstants.LOWER_MOTOR_SPEED;
    upperSpeed = IndexerConstants.UPPER_MOTOR_SPEED_PLACING;
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

  /*
When shooter is up to speed, both indexer motors feeds balls forward into the shooter.
  else if lower sensor detects ball & upper sensor doesn't detect a ball, run both indexer motors upto upper sensor
  else if intake is running & upper sensor detects a ball and the lower doesn't detect a ball, run lower index motor forwards only
  else if intake is not running & upper sensor detects & ball and the lower doesn't detect a ball, don't run indexer
  else if intake is running & neither sensor detects a ball, run both index motor forwards
  else if intake is not running & neither sensor detects a ball, don't run indexer
  else if intake is running reversed, run both index motors backwards
  else if both sensors detect a ball, stop index motors




  
*/
  @Override
  public void periodic() {
    lowerMotor.set(ControlMode.PercentOutput, lowerSpeed);
    upperMotor.set(ControlMode.PercentOutput, upperSpeed);
    lowerSpeed = 0;
    upperSpeed = 0;

//     if "shooter is up to speed" {
//       "run both indexer motors forward"
//     }
//     if else isLowerTriggered() "but upper sensor isn't triggered"{
//       "run both indexer motors until upper sensor is triggered"
//     }
//     if else "intake is running and" isUpperTriggered() "but not lower sensor" {
//       "run lower indexer motors only"
//     }
//     if else "intake is not running and" isUpperTriggered() "but not lower sensor" {
//       "stop both indexer motors"
//     }
//     if else "intake is running and neither sensor detects a ball" {
//       "run both indexer motors forward"
//     }
//     if else "intake is not running and neither sensor detects a ball" {
//       "stop both indexer motors"
//     }
//     if else "intake is running backwards"{
//       "run both indexer motors backwards"
//     }
//     else {
//       "stop both indexer motors"
//     }
   }
 }
