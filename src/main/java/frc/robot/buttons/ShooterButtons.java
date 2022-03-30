package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterButtons {
    public static void Configure(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        final JoystickButton tarmacButton = ControllerConstants.BEHIND_TARMAC_SHOT;
        final JoystickButton hubButton = ControllerConstants.FRONT_OF_HUB_SHOT;
        final JoystickButton launchPadButton = ControllerConstants.LAUNCH_PAD_SHOT;
        final JoystickButton chuckButton = ControllerConstants.CHUCK_IT_SHOT;

        tarmacButton.whileHeld(new ShooterCommand(shooter, indexer, ShooterState.TarmacEdge));
        hubButton.whileHeld(new ShooterCommand(shooter, indexer, ShooterState.Hub));
        launchPadButton.whileHeld(new ShooterCommand(shooter, indexer, ShooterState.LaunchPad));
        chuckButton.whileHeld(new ShooterCommand(shooter, indexer, ShooterState.BallCircle));
    }
}
