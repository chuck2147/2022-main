// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Controller;
import frc.robot.Controller.Button;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberButtons {
    public static void Configure(ClimberSubsystem lowClimber, ClimberSubsystem highClimber, Controller controller) {
        //Low = Medium Bar
        //High = High Bar
        final JoystickButton extendLowJoystickButton = controller.getButton(Controller.Button.A);
        final JoystickButton retractLowJoystickButton = controller.getButton(Controller.Button.B);
        final JoystickButton extendHighJoystickButton = controller.getButton(Controller.Button.X);
        final JoystickButton retractHighJoystickButton = controller.getButton(Controller.Button.Y);

        extendLowJoystickButton.whileHeld(lowClimber::extend, lowClimber);
        retractLowJoystickButton.whileHeld(lowClimber::retract, lowClimber);
        extendHighJoystickButton.whileHeld(highClimber::extend, highClimber);
        retractHighJoystickButton.whileHeld(highClimber::retract, highClimber);
        
     
    }
}
