// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoPathConstants.PathType;
import frc.robot.buttons.ClimberButtons;
import frc.robot.buttons.DriverButtons;
import frc.robot.buttons.IndexerButtons;
import frc.robot.buttons.ShooterButtons;
import frc.robot.commands.Autonomous.Routines.FiveBallAutoCommand;
import frc.robot.commands.Autonomous.Routines.FourBallAutoCommand;
import frc.robot.commands.Autonomous.Routines.ShootAndTaxiPathCommand;
import frc.robot.commands.Autonomous.Routines.ThreeBallTarmacCommand;
import frc.robot.commands.Autonomous.Routines.TwoBallAutoCommand;
import frc.robot.subsystems.ClimberHighSubsystem;
import frc.robot.subsystems.ClimberLowSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final ClimberLowSubsystem lowClimber = new ClimberLowSubsystem();
  private final ClimberHighSubsystem highClimber = new ClimberHighSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem(intake);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutoSelector();
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);

  }

  private void configureAutoSelector() {
    autoChooser.addOption("2 Ball", new TwoBallAutoCommand(drivetrain, vision, shooter, intake, indexer));
    autoChooser.setDefaultOption("4 Ball", new FourBallAutoCommand(drivetrain, vision, shooter, intake, indexer));    
    //autoChooser.addOption("5 Ball", new FiveBallAutoCommand(drivetrain, vision, shooter, intake, indexer));
    
    //autoChooser.addOption("Shoot and Taxi", new ShootAndTaxiPathCommand(drivetrain, vision, shooter, intake, indexer));

    SmartDashboard.putData("Auto Selector" , autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    ClimberButtons.Configure(lowClimber, highClimber);
    ShooterButtons.Configure(shooter, indexer);
    IndexerButtons.Configure(intake);
    DriverButtons.Configure(drivetrain, vision, shooter, indexer);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new DriveForwardCommand(drivetrain);
    return autoChooser.getSelected();
  }
}
