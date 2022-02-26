// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import frc.robot.AxisTrigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class IndexerButtons {
    public static void Configure(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, VisionSubsystem vision) {
        final AxisTrigger collectButton = ControllerConstants.RUN_COLLECT_INTAKE;
        final AxisTrigger reverseButton = ControllerConstants.RUN_INTAKE_REVERSE;

        reverseButton.whileHeld(intake::runIntakeReverse, intake);
        collectButton.whileHeld(intake::runIntakeForward, intake);
    }
}
