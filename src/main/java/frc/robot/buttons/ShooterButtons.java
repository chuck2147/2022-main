package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterButtons {
    public static void Configure(ShooterSubsystem shooter) {
        final JoystickButton tarmacButton = ControllerConstants.BEHIND_TARMAC_SHOT;
        final JoystickButton hubButton = ControllerConstants.FRONT_OF_HUB_SHOT;
        final JoystickButton launchPadButton = ControllerConstants.LAUNCH_PAD_SHOT;
        final JoystickButton chuckButton = ControllerConstants.CHUCK_IT_SHOT;

        tarmacButton.whileHeld(shooter::shootFromBehindTarmac, shooter);
        hubButton.whileHeld(shooter::shootFromFrontOfHub, shooter);
        launchPadButton.whileHeld(shooter::shootFromLaunchPad, shooter);
        chuckButton.whileHeld(shooter::shootChuckIt, shooter);
    }
}
