// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.vision;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Add your docs here. */
public class VisionDrive {
    
    public static void AlignAndDrive(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem, DoubleSupplier speedXSupplier, DoubleSupplier speedYSupplier) {
        double pidRotationVelocity = visionSubsystem.GetRotationVelocityToTarget();
        drivetrain.drive(speedXSupplier.getAsDouble(), speedYSupplier.getAsDouble(), pidRotationVelocity);
    }
}
