// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerButtons {
    public static void Configure(IndexerSubsystem indexer) {
        final JoystickButton indexButton = ControllerConstants.INDEX_IN;

        indexButton.whileHeld(indexer::runIndexer, indexer );
        indexButton.whileHeld(indexer::runHopper, indexer );
    }
    
}
