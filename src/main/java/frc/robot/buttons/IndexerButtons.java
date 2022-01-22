// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Controller;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerButtons {
    public static void Configure(IndexerSubsystem indexer, Controller controller) {
        final JoystickButton button = controller.getButton(Controller.Button.LeftBumper);

        button.whileHeld(indexer::runIndexer, indexer );
        button.whileHeld(indexer::runHopper, indexer );
    }
    
}
