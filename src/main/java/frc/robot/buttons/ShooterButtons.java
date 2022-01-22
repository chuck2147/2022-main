package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Controller;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterButtons {
    public static void Configure(ShooterSubsystem shooter, Controller controller) {
        final JoystickButton tarmacButton = controller.getButton(Controller.Button.A);
        final JoystickButton hubButton = controller.getButton(Controller.Button.B);
        final JoystickButton launchPadButton = controller.getButton(Controller.Button.X);
        final JoystickButton chuckButton = controller.getButton(Controller.Button.Y);

        tarmacButton.whileHeld(shooter::shootFromBehindTarmac, shooter);
        hubButton.whileHeld(shooter::shootFromFrontOfHub, shooter);
        launchPadButton.whileHeld(shooter::shootFromLaunchPad, shooter);
        chuckButton.whileHeld(shooter::shootChuckIt, shooter);
    }
}
