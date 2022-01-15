package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    public IndexerSubsystem() {
        TalonFX indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID); 


        
    }
}
