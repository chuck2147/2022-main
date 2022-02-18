// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;


import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimberButtons {
    public static void Configure(ClimberSubsystem leftSubsystem, ClimberSubsystem rightSubsystem, IntakeSubsystem intakeSubsystem) {
        rightSubsystem.setDefaultCommand(new ClimberCommand(rightSubsystem, intakeSubsystem, ControllerConstants.CLIMB_RIGHT_AXIS, ClimberConstants.RIGHT_CLIMBER_ENCODER_TOP));
        leftSubsystem.setDefaultCommand(new ClimberCommand(leftSubsystem, intakeSubsystem, ControllerConstants.CLIMB_LEFT_AXIS, ClimberConstants.LEFT_CLIMBER_ENCODER_TOP));
    } 
}
