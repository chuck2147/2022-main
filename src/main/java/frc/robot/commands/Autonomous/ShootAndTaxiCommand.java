// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.ShootByVisionCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ShootAndTaxiCommand extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public ShootAndTaxiCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter) {
    addRequirements(drivetrain, visionSubsystem, shooter);

    addCommands(
      new ShootByVisionCommand(drivetrain, visionSubsystem, shooter, () -> 0, () -> 0), 
      new InstantCommand(() -> drivetrain.resetOdometry(TrajectoryConstants.GO_BACKWARDS_TRAJECTORY.getInitialPose())),
      AutoDriveBaseCommand.GetCommand(drivetrain, TrajectoryConstants.GO_BACKWARDS_TRAJECTORY)
    );
  }
}