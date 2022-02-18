package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardCommand extends CommandBase{
    DrivetrainSubsystem drivetrainSubsystem;
    private double endTime;
    public DriveForwardCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + 1.5;
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(-0.5, 0,  0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
    }
}
