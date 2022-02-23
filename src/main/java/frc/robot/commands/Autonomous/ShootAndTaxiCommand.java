// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    var startPose = new Pose2d(0, 0, new Rotation2d(0));

    addCommands(
      new ResetOdemetryCommand(drivetrain, startPose),
      //new ShootByVisionCommand(drivetrain, visionSubsystem, shooter, () -> 0, () -> 0), 
      new AutoDistanceRotateCommand(drivetrain, 0, -1.5, 0)
    );
  }
}
