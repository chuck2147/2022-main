package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AlignShooterSideCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class VisionButtons {
    public static void Configure(DrivetrainSubsystem drivetrain) {
        final JoystickButton visionAlign = ControllerConstants.VISION_ALIGN;

        visionAlign.whileHeld(new AlignShooterSideCommand(drivetrain));
    }
}
