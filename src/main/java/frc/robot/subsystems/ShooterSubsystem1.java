package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterHoodMotor = new TalonFX(ShooterConstants.HOOD_SHOOTER_MOTOR_ID);
    TalonFX shooterLowerMotor = new TalonFX(ShooterConstants.LOWER_SHOOTER_MOTOR_ID);
    TalonFX shooterUpperMotor = new TalonFX(ShooterConstants.UPPER_SHOOTER_MOTOR_ID);
    public  ShooterSubsystem() {
    
    }

    
}
