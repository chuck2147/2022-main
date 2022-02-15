// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignShooterSideCommand extends CommandBase {
  DrivetrainSubsystem drivetrain;
  VisionSubsystem visionSubsystem;
  DoubleSupplier speedXSupplier;
  DoubleSupplier speedYSupplier;
  
  public AlignShooterSideCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, DoubleSupplier speedXSupplier, DoubleSupplier speedYSupplier) {
    addRequirements(drivetrain, visionSubsystem);
    this.drivetrain = drivetrain;
    this.visionSubsystem = visionSubsystem;
    this.speedXSupplier = speedXSupplier;
    this.speedYSupplier = speedYSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}  

  @Override
  public void execute() {
    double pidAngularVelocity = visionSubsystem.GetRotationVelocityToTarget();
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedXSupplier.getAsDouble(), speedYSupplier.getAsDouble(), pidAngularVelocity, drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return visionSubsystem.IsOnTarget();
  }

}
