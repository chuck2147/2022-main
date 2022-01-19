package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Controller;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;

public class ShooterButtons {
    public static void Configure(ShooterSubsystem subsystem, Controller controller) {
        final JoystickButton tarmacButton = controller.getButton(Controller.Button.A);
        final JoystickButton hubButton = controller.getButton(Controller.Button.B);
        final JoystickButton launchPadButton = controller.getButton(Controller.Button.X);
        final JoystickButton chuckButton = controller.getButton(Controller.Button.Y);
        tarmacButton.whileHeld(new ShooterCommand(subsystem, ShooterState.Tarmac));
        hubButton.whileHeld(new ShooterCommand(subsystem, ShooterState.Hub));
        launchPadButton.whileHeld(new ShooterCommand(subsystem, ShooterState.LaunchPad));
        chuckButton.whileHeld(new ShooterCommand(subsystem, ShooterState.ChuckIt));
    }
}
