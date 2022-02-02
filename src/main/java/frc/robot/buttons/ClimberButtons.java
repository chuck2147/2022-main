// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberButtons {
    public static void Configure(ClimberSubsystem lowClimber, ClimberSubsystem highClimber) {
        //Low = Medium Bar
        //High = High Bar
        final JoystickButton extendLowJoystickButton = ControllerConstants.CLIMB_LOW_UP_BUTTON;
        final JoystickButton retractLowJoystickButton = ControllerConstants.CLIMB_LOW_DOWN_BUTTON;
        final JoystickButton extendHighJoystickButton = ControllerConstants.CLIMB_HIGH_UP_BUTTON;
        final JoystickButton retractHighJoystickButton = ControllerConstants.CLIMB_HIGH_DOWN_BUTTON;

        extendLowJoystickButton.whileHeld(lowClimber::extend, lowClimber);
        retractLowJoystickButton.whileHeld(lowClimber::retract, lowClimber);
        extendHighJoystickButton.whileHeld(highClimber::extend, highClimber);
        retractHighJoystickButton.whileHeld(highClimber::retract, highClimber);
        
     
    }
}
