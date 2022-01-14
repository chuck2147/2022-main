// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controller.Hand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.TankDriveCommand;
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
  private final DrivetrainSubsystem Drivetrain = new DrivetrainSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final Controller driverController = new Controller(0, Hand.Left, 0.05);
  private final Controller operatorController = new Controller(1, Hand.Left, 0.05);

  //private final JoystickButton climberUpButton = driverController.getButton(Controller.Button.A);
  //private final JoystickButton shootCloseButton = operatorController.getButton(Controller.Button.A);
  private final JoystickButton shooterTriangleButton = operatorController.getButton(Controller.Button.X);
  private final JoystickButton shooterBehindLineButton = operatorController.getButton(Controller.Button.A);
  private final JoystickButton shooterFarButton = operatorController.getButton(Controller.Button.B);
  private final JoystickButton shooterFrontOfTrench = operatorController.getButton(Controller.Button.Y);
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

    Drivetrain.setDefaultCommand(getArcadeDrive());

    shooterBehindLineButton.whileHeld(shooter::shootFromBehindLine, shooter);
    shooterBehindLineButton.whenReleased(shooter::stopShooter, shooter);
    
    shooterTriangleButton.whileHeld(shooter::shootFromTriangle, shooter);
    shooterTriangleButton.whenReleased(shooter::stopShooter, shooter);

    shooterFrontOfTrench.whileHeld(shooter::shootFromFrontOfTrench, shooter);
    shooterFrontOfTrench.whenReleased(shooter::stopShooter, shooter);
    shooterFarButton.whileHeld(shooter::shootFromFar, shooter);
    shooterFarButton.whenReleased(shooter::stopShooter, shooter);
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new DriveForwardCommand();
  }

  private Command getTankDrive() {
    return new TankDriveCommand(Drivetrain, () -> driverController.getRawAxis(5), () -> driverController.getRawAxis(0));
 }

 private Command getArcadeDrive() {
    return new ArcadeDriveCommand(Drivetrain, () -> -driverController.getRawAxis(1), () -> driverController.getRawAxis(4));
 }

}
