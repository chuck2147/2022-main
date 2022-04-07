// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Routines;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants.BallCount;
import frc.robot.commands.Autonomous.AutoPathPlanCommand;
import frc.robot.commands.Autonomous.AutoShootCommand;
import frc.robot.commands.Autonomous.ResetOdometryCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class Simple extends SequentialCommandGroup {
  /** Creates a new ShootAndTaxiCommand. */
  public Simple(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer) {
    addRequirements(drivetrain, visionSubsystem, shooter, intake, indexer);
    
    var lowerSpeed = ShooterConstants.TARMAC_EDGE_LOWER.value; 
    var upperSpeed = ShooterConstants.TARMAC_EDGE_UPPER.value;

    addCommands(
      new PerpetualCommand(new RunCommand(()-> {
        drivetrain.driveRobotCentric(-0.8, 0, 0);
      }, drivetrain)).withTimeout(2.5),
      new AutoShootCommand(drivetrain, visionSubsystem, shooter, indexer, BallCount.One, lowerSpeed, upperSpeed, true).withTimeout(3)
    );
  }
}
