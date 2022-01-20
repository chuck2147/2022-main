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

        public void stopIndexer(){
            indexerMotor.set(ControlMode.Velocity, 0);
        }
    
        public void stopHopper(){
            hopperMotor.set(ControlMode.Velocity, 0);
        }

        public void runIndexer(){
            indexerMotor.set(ControlMode.Velocity, IndexerConstants.INDEXER_MOTOR_SPEED);
        }

        public void runHopper(){
            hopperMotor.set(ControlMode.Velocity, IndexerConstants.HOPPER_MOTOR_SPEED);
        }

        public void inverseIndexer(){
            indexerMotor.set(ControlMode.Velocity, -IndexerConstants.INDEXER_MOTOR_SPEED);
        }

        public void inverseHopper(){
            hopperMotor.set(ControlMode.Velocity, -IndexerConstants.HOPPER_MOTOR_SPEED);
        }

        public void manualIndexer(){
            indexerMotor.set(ControlMode.Velocity, IndexerConstants.INDEXER_MOTOR_SPEED);
            hopperMotor.set(ControlMode.Velocity, IndexerConstants.HOPPER_MOTOR_SPEED);
        }

        public void manualInverseIndexer(){
            indexerMotor.set(ControlMode.Velocity, -IndexerConstants.INDEXER_MOTOR_SPEED);
            hopperMotor.set(ControlMode.Velocity, -IndexerConstants.HOPPER_MOTOR_SPEED);
        }

        public void manualStopIndexer(){
            indexerMotor.set(ControlMode.Velocity, 0);
            hopperMotor.set(ControlMode.Velocity, 0);
        }

        public boolean isIndexerTriggered(){
            return irIndexer.getVoltage() > IndexerConstants.INDEXER_IR_VOLTAGE; 
        }

        public boolean isHopperTriggered(){
            return irHopper.getVoltage() > IndexerConstants.HOPPER_IR_VOLTAGE;
        }

        @Override
        public void periodic(){

        }
    }
