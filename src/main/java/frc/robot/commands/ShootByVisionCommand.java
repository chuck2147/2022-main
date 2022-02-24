// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.vision.VisionShooting;

public class ShootByVisionCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private ShooterSubsystem shooterSubsystem;
  private DoubleSupplier speedXSupplier;
  private DoubleSupplier speedYSupplier;
  private VisionShooting visionShooting;

  
  public ShootByVisionCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, DoubleSupplier speedXSupplier, DoubleSupplier speedYSupplier) {
    addRequirements(drivetrain, visionSubsystem, shooter);
    this.drivetrain = drivetrain;
    this.shooterSubsystem = shooter;
    this.speedXSupplier = speedXSupplier;
    this.speedYSupplier = speedYSupplier;

    visionShooting = new VisionShooting(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.SetOverrideVision(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionShooting.AlignAndDrive(drivetrain, speedXSupplier, speedYSupplier);

    visionShooting.ShootByDistance(shooterSubsystem);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
