package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDNTValue;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX upperMotor = new TalonFX(ShooterConstants.UPPER_MOTOR_ID);
  private final TalonFX lowerMotor = new TalonFX(ShooterConstants.LOWER_MOTOR_ID);
  private double lowerTargetSpeed = 0;
  private double upperTargetSpeed = 0;

  ShuffleboardTab tab = Shuffleboard.getTab("NTValues");
  NetworkTableEntry upperVelocityGraphEntry = tab.add("Upper Current Velocity Graph", 0)
    .withSize(2, 1)
    .withWidget(BuiltInWidgets.kGraph)
    .getEntry();
    NetworkTableEntry upperVelocityEntry = tab.add("Upper Current Velocity", 0)
    .withSize(2, 1)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();
  NetworkTableEntry lowerVelocityEntry = tab.add("Lower Current Velocity", 0)
    .withSize(2, 1)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();
  
  public ShooterSubsystem() {
    upperMotor.configFactoryDefault();
    lowerMotor.configFactoryDefault();
  
    upperMotor.setNeutralMode(NeutralMode.Coast);
    lowerMotor.setNeutralMode(NeutralMode.Coast);
  
    upperMotor.setInverted(TalonFXInvertType.CounterClockwise);
    lowerMotor.setInverted(TalonFXInvertType.CounterClockwise);

    // "full output" will now scale to 12 Volts for all control modes when enabled.
    upperMotor.configVoltageCompSaturation(12);
    lowerMotor.configVoltageCompSaturation(12); 
    lowerMotor.enableVoltageCompensation(true);
    upperMotor.enableVoltageCompensation(true);
    
		/* Config neutral deadband to be the smallest possible */
    upperMotor.configNeutralDeadband(0.001);
    lowerMotor.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] (Talon integrated encoder, PID_Slot, timeouts)*/
    upperMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    lowerMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
   
    // PIDF
    new PIDNTValue(ShooterConstants.UPPER_P, ShooterConstants.UPPER_I, ShooterConstants.UPPER_D, ShooterConstants.UPPER_F, upperMotor, "Upper Shooter"); 
    new PIDNTValue(ShooterConstants.LOWER_P, ShooterConstants.LOWER_I, ShooterConstants.LOWER_D, ShooterConstants.LOWER_F, lowerMotor, "Lower Shooter"); 
  }

  public static double encToRPM(double enc) {
    return enc / 100 * 1000 / 2048 * 60;
  }

  public static double RPMToEnc(double rpm) {
    return rpm * 100 / 1000 * 2048 / 60;
  }
  
  public double getVelocityUpper() {
    return upperMotor.getSelectedSensorVelocity();
  }

  public double getVelocityLower() {
    return lowerMotor.getSelectedSensorVelocity();
  }

  public void stopShooter(){
    upperMotor.set(ControlMode.PercentOutput, 0);
    lowerMotor.set(ControlMode.PercentOutput, 0);
  }

  // Lower_Motor Velocity will always take longer to get on target... so only needs lower velocity
  public boolean isUpToSpeed() {
    boolean isShooterAtSpeed = false;
    boolean isShooting = lowerTargetSpeed != 0 || upperTargetSpeed != 0;

    if (isShooting) {
      boolean lowerOnTarget = withinTolerance(lowerTargetSpeed, getVelocityLower(), ShooterConstants.velocityPIDTolerance);
      boolean upperOnTarget = withinTolerance(upperTargetSpeed, getVelocityUpper(), ShooterConstants.velocityPIDTolerance);
      isShooterAtSpeed = lowerOnTarget && upperOnTarget;
    }

    return isShooterAtSpeed;
  }

  public void setSpeeds(double lowerSpeed, double upperSpeed) {
    lowerMotor.set(TalonFXControlMode.Velocity, lowerSpeed);
    upperMotor.set(TalonFXControlMode.Velocity, upperSpeed);
  }
 
  @Override
  public void periodic() {
    upperVelocityEntry.setValue(upperMotor.getSelectedSensorVelocity());
    upperVelocityGraphEntry.setValue(lowerMotor.getSelectedSensorVelocity());
    lowerVelocityEntry.setValue(lowerMotor.getSelectedSensorVelocity());
  }

  private boolean withinTolerance(double firstValue, double secondValue, double acceptedTolerance) {
    return Math.abs(firstValue - secondValue) <= acceptedTolerance;
  }
}
