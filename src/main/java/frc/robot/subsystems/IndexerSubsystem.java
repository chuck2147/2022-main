package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private AnalogInput irIndexer;
    private AnalogInput irHopper;

    public IndexerSubsystem() {
        TalonFX indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID); 
        
        irIndexer = new AnalogInput(IndexerConstants.INDEXER_IR_ID);
        irHopper = new AnalogInput(IndexerConstants.HOPPER_IR_ID);
        
    }
}
