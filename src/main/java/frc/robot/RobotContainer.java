// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.buttons.ClimberButtons;
import frc.robot.buttons.IndexerButtons;
import frc.robot.buttons.IntakeButtons;
import frc.robot.buttons.ShooterButtons;
import frc.robot.commands.Autonomous.DriveForwardCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final DrivetrainSubsystem Drivetrain = DrivetrainSubsystem.getInstance();
  // private final ClimberSubsystem lowClimber = new ClimberSubsystem(ClimberConstants.LOW_CLIMBER_MOTOR_ID, ClimberConstants.CLIMBER_LOW_AIR_IN, ClimberConstants.CLIMBER_LOW_AIR_OUT);
  // private final ClimberSubsystem highClimber = new ClimberSubsystem(ClimberConstants.HIGH_CLIMBER_MOTOR_ID, ClimberConstants.CLIMBER_HIGH_AIR_IN, ClimberConstants.CLIMBER_HIGH_AIR_OUT);
  // private final IndexerSubsystem indexer = new IndexerSubsystem();
  // private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final Controller driverController = new Controller(0, 0.05);
  private final Controller operatorController = new Controller(1, 0.05);


  //private final JoystickButton climberUpButton = driverController.getButton(Controller.Button.A);
  // private final JoystickButton shooterFrontOfHubButton = operatorController.getButton(Controller.Button.X);
  // private final JoystickButton shooterBehindTarmacButton = operatorController.getButton(Controller.Button.A);
  // private final JoystickButton shooterChuckItButton = operatorController.getButton(Controller.Button.B);
  // private final JoystickButton shooterLaunchPadButton = operatorController.getButton(Controller.Button.Y);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // shooterBehindTarmacButton.whileHeld(shooter::shootFromBehindTarmac, shooter);
    // shooterFrontOfHubButton.whileHeld(shooter::shootFromFrontOfHub, shooter);
    // shooterLaunchPadButton.whileHeld(shooter::shootFromLaunchPad, shooter);
    // shooterChuckItButton.whileHeld(shooter::shootChuckIt, shooter);
    
    // IntakeButtons.Configure(intake, operatorController);
    // ClimberButtons.Configure(lowClimber, highClimber, driverController);
    ShooterButtons.Configure(shooter, operatorController);
    // IndexerButtons.Configure(indexer, driverController);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new DriveForwardCommand();
  }
}
