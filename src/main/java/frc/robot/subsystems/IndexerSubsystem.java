package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private AnalogInput irIndexer = new AnalogInput(IndexerConstants.INDEXER_IR_ID);
    private AnalogInput irHopper = new AnalogInput(IndexerConstants.HOPPER_IR_ID);
    TalonFX indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID); 
    TalonFX hopperMotor = new TalonFX(IndexerConstants.HOPPER_MOTOR_ID);
    double indexerSpeed = 0;
    double hopperSpeed = 0;

        public void stopIndexer(){
            indexerSpeed = 0;
        }
    
        public void stopHopper(){
            hopperSpeed = 0;
        }

        public void runIndexer(){
            indexerSpeed = IndexerConstants.INDEXER_MOTOR_SPEED;
        }

        public void runHopper(){
            hopperSpeed = IndexerConstants.HOPPER_MOTOR_SPEED;
        }

        public void inverseIndexer(){
            indexerSpeed = -IndexerConstants.INDEXER_MOTOR_SPEED;
        }

        public void inverseHopper(){
           hopperSpeed = -IndexerConstants.HOPPER_MOTOR_SPEED;
        }

        public void manualIndexer(){
            indexerSpeed = IndexerConstants.INDEXER_MOTOR_SPEED;
            hopperSpeed = IndexerConstants.HOPPER_MOTOR_SPEED;
        }

        public void manualInverseIndexer(){
            indexerSpeed = -IndexerConstants.INDEXER_MOTOR_SPEED;
            hopperSpeed = -IndexerConstants.HOPPER_MOTOR_SPEED;
        }

        public void manualStopIndexer(){
            indexerSpeed = 0;
            hopperSpeed = 0;
        }

        public boolean isIndexerTriggered(){
            return irIndexer.getVoltage() > IndexerConstants.INDEXER_IR_VOLTAGE; 
        }

        public boolean isHopperTriggered(){
            return irHopper.getVoltage() > IndexerConstants.HOPPER_IR_VOLTAGE;
        }

        @Override
        public void periodic(){
            indexerMotor.set(ControlMode.PercentOutput, indexerSpeed);
            hopperMotor.set(ControlMode.PercentOutput, hopperSpeed);
        }
    }
