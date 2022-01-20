// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Controller;
import frc.robot.commands.IndexerCommand;
import frc.robot.subsystems.IndexerSubsystem;

/** Add your docs here. */
public class IndexerButtons {
    public static void Configure(IndexerSubsystem subsystem, Controller controller) {
        final JoystickButton button = controller.getButton(Controller.Button.LeftBumper);
        button.whileHeld(new IndexerCommand(subsystem));
    }
    
}
