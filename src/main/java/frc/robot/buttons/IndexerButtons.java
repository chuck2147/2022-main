// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IndexerConstants.IndexerStates;
import frc.robot.commands.IndexerCommand;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerButtons {
    public static void Configure(IndexerSubsystem indexer) {
        final JoystickButton bottomButton = ControllerConstants.INDEX_OUT;
        final JoystickButton topButton = ControllerConstants.INDEX_IN;

        // leftButton.whileHeld(indexer::manualIndexer, indexer );
        // rightButton.whileHeld(indexer::manualInverseIndexer, indexer );
        // leftButton.whenReleased(indexer::manualStopIndexer, indexer);
        // rightButton.whenReleased(indexer::manualStopIndexer, indexer);
        // bottomButton.whileHeld(new IndexerCommand(indexer, IndexerStates.backward));
        // topButton.whileHeld(new IndexerCommand(indexer, IndexerStates.forward));
    }
    
}
