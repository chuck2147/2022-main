// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.vision.VisionShooting;

public class ShootByVisionCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private ShooterSubsystem shooter;  
  private IndexerSubsystem indexer;
  private DoubleSupplier speedXSupplier;
  private DoubleSupplier speedYSupplier;
  private VisionShooting visionShooting;

  
  public ShootByVisionCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
                              DoubleSupplier speedXSupplier, DoubleSupplier speedYSupplier) {
    addRequirements(drivetrain, visionSubsystem, shooterSubsystem, indexerSubsystem);
    this.drivetrain = drivetrain;
    this.shooter = shooterSubsystem;
    this.indexer = indexerSubsystem;
    this.speedXSupplier = speedXSupplier;
    this.speedYSupplier = speedYSupplier;

    visionShooting = new VisionShooting(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionShooting.AlignAndDrive(drivetrain, speedXSupplier, speedYSupplier);

    visionShooting.ShootByDistance(shooter, indexer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
