// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TankDriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final Supplier<Double> rightSpeedSupplier;
  private final Supplier<Double> leftSpeedSupplier;

  /** Creates a new TankDriveCommand. */
  public TankDriveCommand(
        DrivetrainSubsystem drivetrain, 
        Supplier<Double> rightSpeedSupplier, Supplier<Double> leftSpeedSupplier) {
    this.drivetrain = drivetrain;
    this.rightSpeedSupplier = rightSpeedSupplier;
    this.leftSpeedSupplier = leftSpeedSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.drivetrain.tankDrive(this.rightSpeedSupplier.get(), this.leftSpeedSupplier.get());
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
