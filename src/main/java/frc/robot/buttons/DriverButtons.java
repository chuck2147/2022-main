package frc.robot.buttons;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AlignShooterSideCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ShootByVisionCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriverButtons {
    public static void Configure(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        final JoystickButton visionAlign = ControllerConstants.VISION_ALIGN;
        final JoystickButton resetGyro = ControllerConstants.RESET_GYRO;
        final JoystickButton resetPose = ControllerConstants.RESET_POSE;
        final DoubleSupplier driveX = ControllerConstants.DRIVE_X;
        final DoubleSupplier driveY = ControllerConstants.DRIVE_Y;
        final DoubleSupplier driveRotation = ControllerConstants.DRIVE_ROTATION;
        final JoystickButton visionAndShootAlign = ControllerConstants.VISION_AND_SHOOTER_ALIGN;

        drivetrain.setDefaultCommand(new DefaultDriveCommand(
            drivetrain, driveY, driveX, driveRotation));

        visionAlign.whileHeld(new AlignShooterSideCommand(drivetrain, visionSubsystem, driveX, driveY));
        visionAndShootAlign.whileHeld(new ShootByVisionCommand(drivetrain, visionSubsystem, shooter, indexer, driveX, driveY));
        resetGyro.whenPressed(()->drivetrain.resetGyroscope());
        resetPose.whenPressed(()->drivetrain.resetPose(new Vector2d(0, 0), new Rotation2d(0)));
        // private final SlewRateLimiter filter = new SlewRateLimiter(1.5);
    }


}
