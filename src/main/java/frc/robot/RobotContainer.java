// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.buttons.ClimberButtons;
import frc.robot.buttons.IndexerButtons;
import frc.robot.buttons.ShooterButtons;
import frc.robot.commands.CollectCommand;
import frc.robot.commands.DefaultDriveCommand;
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
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  // private final ClimberSubsystem lowClimber = new ClimberSubsystem(ClimberConstants.LOW_CLIMBER_MOTOR_ID, ClimberConstants.CLIMBER_LOW_AIR_IN, ClimberConstants.CLIMBER_LOW_AIR_OUT);
  // private final ClimberSubsystem highClimber = new ClimberSubsystem(ClimberConstants.HIGH_CLIMBER_MOTOR_ID, ClimberConstants.CLIMBER_HIGH_AIR_IN, ClimberConstants.CLIMBER_HIGH_AIR_OUT);
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final Controller driverController = new Controller(0, 0.05);
  private final Controller operatorController = new Controller(1, 0.05);
  private final SlewRateLimiter filter = new SlewRateLimiter(1.5);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new DefaultDriveCommand(
            drivetrain,
            () -> -modifyAxis(driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            //filter
    ));
    indexer.setDefaultCommand(
      new CollectCommand(indexer, intake, shooter, new AxisTrigger(operatorController, 3))
    );
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
    
     // ClimberButtons.Configure(lowClimber, highClimber, driverController);
     ShooterButtons.Configure(shooter);
     IndexerButtons.Configure(indexer);

    driverController.getButton(Controller.Button.Start)
      .whenPressed(()->drivetrain.resetGyroscope());
    driverController.getButton(Controller.Button.Back)
      .whenPressed(()->drivetrain.resetPose(new Vector2d(0, 0), new Rotation2d(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new DriveForwardCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
