package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    public DrivetrainSubsystem () {

    }
    public static DrivetrainSubsystem getInstance() {
        return instance;
      }
}
