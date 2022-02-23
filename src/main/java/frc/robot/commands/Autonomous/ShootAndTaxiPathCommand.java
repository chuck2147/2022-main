// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ShootAndTaxiPathCommand extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public ShootAndTaxiPathCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter) {
    addRequirements(drivetrain, visionSubsystem, shooter);

    PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("Taxi", AutoPathConstants.kMaxSpeedMetersPerSecond, AutoPathConstants.kMaxAccelerationMetersPerSecondSquared);
    //drivetrain.resetOdometry(pathTrajectory.getInitialPose());

    addCommands(
      new ResetOdemetryCommand(drivetrain, pathTrajectory.getInitialPose()),
      //new ShootByVisionCommand(drivetrain, visionSubsystem, shooter, () -> 0, () -> 0),
      AutoPathPlanCommand.GetCommand(drivetrain, pathTrajectory)
    );
  }
}
