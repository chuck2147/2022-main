// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ClimberHookCommand;
import frc.robot.subsystems.ClimberHighSubsystem;
import frc.robot.subsystems.ClimberLowSubsystem;

public class ClimberButtons {

    public static void Configure(ClimberLowSubsystem lowSubsystem, ClimberHighSubsystem highSubsystem) {
        
        final JoystickButton hookButton = ControllerConstants.HOOK_PISTON;

        lowSubsystem.setDefaultCommand(new ClimberCommand(lowSubsystem, ControllerConstants.CLIMB_LOW_AXIS));
        highSubsystem.setDefaultCommand(new ClimberCommand(highSubsystem, ControllerConstants.CLIMB_HIGH_AXIS));
        
        hookButton.whileHeld(new ClimberHookCommand(lowSubsystem));
    } 
}
