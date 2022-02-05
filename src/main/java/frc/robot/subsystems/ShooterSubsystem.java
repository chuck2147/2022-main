package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private boolean isShooterAtSpeed = false;

  public enum ShooterDistances {
    BEHIND_TRENCH, FRONT_OF_TRENCH, BEHIND_LINE
  }

  public enum HoodMovements {
    UP, DOWN
  }

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
    lowerMotor.setInverted(TalonFXInvertType.Clockwise);

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
    return -lowerMotor.getSelectedSensorVelocity();
  } 
  public void stopShooter(){
    upperMotor.set(ControlMode.PercentOutput, 0);
    lowerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void shootFromBehindTarmac() {
    lowerTargetSpeed = ShooterConstants.BEHIND_TARMAC_LOWER.value;
    upperTargetSpeed = ShooterConstants.BEHIND_TARMAC_UPPER.value;
  }
  public void shootFromFrontOfHub() {
    lowerTargetSpeed = ShooterConstants.FRONT_OF_HUB_LOWER.value;
    upperTargetSpeed = ShooterConstants.FRONT_OF_HUB_UPPER.value;
  }
  public void shootFromLaunchPad(){
    lowerTargetSpeed = ShooterConstants.LAUNCH_PAD_LOWER.value;
    upperTargetSpeed = ShooterConstants.LAUNCH_PAD_UPPER.value;
  }
  public void shootChuckIt() {
    lowerTargetSpeed = ShooterConstants.CHUCK_IT_LOWER.value;
    upperTargetSpeed = ShooterConstants.CHUCK_IT_UPPER.value;
  }
  // Lower_Motor Velocity will always take longer to get on target... so only needs lower velocity
  public boolean isUpToSpeed() {
    return isShooterAtSpeed;
  }
 
  @Override
  public void periodic() {
    upperVelocityEntry.setValue(upperMotor.getSelectedSensorVelocity());
    upperVelocityGraphEntry.setValue(lowerMotor.getSelectedSensorVelocity());
    lowerVelocityEntry.setValue(lowerMotor.getSelectedSensorVelocity());

    if(lowerTargetSpeed == 0){
      lowerMotor.set(TalonFXControlMode.PercentOutput, 0);
    }else {
      lowerMotor.set(TalonFXControlMode.Velocity, -lowerTargetSpeed);
    }

    if (upperTargetSpeed == 0) {
      upperMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      upperMotor.set(TalonFXControlMode.Velocity, upperTargetSpeed);
    }
    boolean isShooting = lowerTargetSpeed != 0 || upperTargetSpeed != 0;
    if (!isShooting) {
      isShooterAtSpeed = false;
    } else {
      System.out.println(lowerTargetSpeed + ", " + getVelocityLower());
      boolean lowerOnTarget = Math.abs(lowerTargetSpeed - getVelocityLower()) <= ShooterConstants.velocityPIDTolerance;
      boolean upperOnTarget = Math.abs(upperTargetSpeed - getVelocityUpper()) <= ShooterConstants.velocityPIDTolerance;
      isShooterAtSpeed = lowerOnTarget && upperOnTarget;
    }
    lowerTargetSpeed = 0;
    upperTargetSpeed = 0;
  }
}
