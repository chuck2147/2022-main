// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AxisTrigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants.IntakeStates;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeButtons {
    public static void Configure(IntakeSubsystem intake) {
        final AxisTrigger intakeButton = ControllerConstants.INTAKE_IN_BUTTON;
        final AxisTrigger reverseintakeButton = ControllerConstants.INTAKE_OUT_BUTTON;

        intakeButton.whileHeld(intake::runIntake, intake);
        intakeButton.whileHeld(intake::retractIntake, intake);
        reverseintakeButton.whileHeld(intake::reverseIntake, intake);
        reverseintakeButton.whileHeld(intake::retractIntake, intake);
        intakeButton.whenReleased(intake::extendIntake, intake);
        reverseintakeButton.whenReleased(intake::extendIntake, intake);
    }
}
