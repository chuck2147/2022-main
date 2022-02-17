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
import frc.robot.util.MathCommon;
import frc.robot.util.interpolableMap.InterpolatingDouble;
import frc.robot.util.vision.VisionDrive;

public class ShootByVisionCommand extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private VisionSubsystem visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private DoubleSupplier speedXSupplier;
  private DoubleSupplier speedYSupplier;

  private double setDistance;
  
  public ShootByVisionCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, ShooterSubsystem shooter, DoubleSupplier speedXSupplier, DoubleSupplier speedYSupplier) {
    addRequirements(drivetrain, visionSubsystem, shooter);
    this.drivetrain = drivetrain;
    this.visionSubsystem = visionSubsystem;
    this.shooterSubsystem = shooter;
    this.speedXSupplier = speedXSupplier;
    this.speedYSupplier = speedYSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.SetOverrideVision(false);
    setDistance = Double.MIN_NORMAL;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VisionDrive.AlignAndDrive(drivetrain, visionSubsystem, speedXSupplier, speedYSupplier);

    var distanceToTarget = visionSubsystem.GetDistanceToTarget();

    // See if the distance has changed before setting the wheel speeds.
    // A tolerance for variability is needed since the distance can change within inches from the Limelight even though it is standing still.
    if (setDistance == Double.MIN_VALUE || !MathCommon.WithinTolerance(distanceToTarget, setDistance, VisionConstants.DISTANCE_FROM_TARGET_TOLERANCE_IN_INCHES)) {
      setDistance = distanceToTarget;

      var interpolatedDistance = new InterpolatingDouble(setDistance);
    
      var lowerTargetSpeed = ShooterConstants.LOWER_SHOOTER_SPEED_MAP.getInterpolated(interpolatedDistance);
      var upperTargetSpeed = ShooterConstants.UPPER_SHOOTER_SPEED_MAP.getInterpolated(interpolatedDistance);

      if (lowerTargetSpeed != null && upperTargetSpeed != null) {
        shooterSubsystem.setSpeeds(lowerTargetSpeed.value, upperTargetSpeed.value);
      }
    }
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
