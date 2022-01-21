package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();
    PigeonIMU pigeon = new PigeonIMU(20);  //CAN Id for gyro
    
    public DrivetrainSubsystem () {

    }
    
    public double getAngularVelocity() {
      double[] angularVelocities = new double[3];
      pigeon.getRawGyro(angularVelocities); 
      return angularVelocities[2];
    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
      }
}
