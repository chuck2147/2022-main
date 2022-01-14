package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);
    
    public ClimberSubsystem() {
        
    }
  
}
