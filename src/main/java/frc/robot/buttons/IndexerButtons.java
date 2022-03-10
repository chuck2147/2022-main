// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import frc.robot.AxisTrigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexerButtons {
    public static void Configure(IntakeSubsystem intake) {
        final AxisTrigger collectButton = ControllerConstants.RUN_COLLECT_INTAKE;
        final AxisTrigger reverseButton = ControllerConstants.RUN_INTAKE_REVERSE;

        reverseButton.whileHeld(new IntakeCommand(intake, IntakeStates.Reverse));
        collectButton.whileHeld(new IntakeCommand(intake, IntakeStates.Forward));
    }
}
