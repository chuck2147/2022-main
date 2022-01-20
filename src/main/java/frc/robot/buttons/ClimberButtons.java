// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Controller;
import frc.robot.Controller.Button;
import frc.robot.Constants.ClimberConstants.ClimberState;
import frc.robot.Constants.ClimberConstants.ClimberType;
import frc.robot.commands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;

/** Add your docs here. */
public class ClimberButtons {
    public static void Configure(ClimberSubsystem subsystem, ClimberType climberType, Controller controller) {
        Button extendButton;
        Button retractButton;
        if (climberType == ClimberType.Low) {
            extendButton = Controller.Button.A;
            retractButton = Controller.Button.B;
        }
        else {
            extendButton = Controller.Button.X;
            retractButton = Controller.Button.Y;
        }
        final JoystickButton extendJoystickButton = controller.getButton(extendButton);
        final JoystickButton retractJoystickButton = controller.getButton(retractButton);

      

        extendJoystickButton.whileHeld(new ClimberCommand(subsystem, ClimberState.Up));
        retractJoystickButton.whileHeld(new ClimberCommand(subsystem, ClimberState.Down));
     
    }
}
