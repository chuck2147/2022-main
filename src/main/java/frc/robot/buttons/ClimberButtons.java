// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;


import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberButtons {
    public static void Configure(ClimberSubsystem leftSubsystem, ClimberSubsystem rightSubsystem) {
        new ClimberCommand(rightSubsystem, ControllerConstants.CLIMB_RIGHT_AXIS);
        new ClimberCommand(leftSubsystem, ControllerConstants.CLIMB_LEFT_AXIS);
    }
}
