// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AxisTrigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ReverseCollectCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexerButtons {
    public static void Configure(IntakeSubsystem intake, IndexerSubsystem indexer) {
        final AxisTrigger reverseButton = ControllerConstants.RUN_INTAKE_REVERSE;

        reverseButton.whileHeld(new ReverseCollectCommand(intake, indexer));
    }
    
}
