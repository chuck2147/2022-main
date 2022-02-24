// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.vision.VisionShooting;

public class AutoShootCommand extends CommandBase {
  
  private DrivetrainSubsystem drivetrain;
  private ShooterSubsystem shooter;
  private IndexerSubsystem indexer; 
  private VisionSubsystem vision;

  private VisionShooting visionShooting;
  
  public AutoShootCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    addRequirements(visionSubsystem, shooterSubsystem, indexerSubsystem);

    visionShooting = new VisionShooting(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionShooting.Align(drivetrain);

    visionShooting.ShootByDistance(shooter);

    if (visionShooting.IsOnTarget() && shooter.isUpToSpeed()) {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
