// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.vision;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.time.StopWatch;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.MathCommon;
import frc.robot.util.interpolableMap.InterpolatingDouble;

/** Add your docs here. */
public class VisionShooting {

    VisionSubsystem visionSubsystem;
    private double setDistance;
    private double lowerTargetSpeed;
    private double upperTargetSpeed;


    public VisionShooting(VisionSubsystem vision) {
        visionSubsystem = vision;
                
        setDistance = Double.NaN;
        lowerTargetSpeed = Double.NaN;        
        upperTargetSpeed = Double.NaN;
    }
    
    public void AlignAndDrive(DrivetrainSubsystem drivetrain, DoubleSupplier speedXSupplier, DoubleSupplier speedYSupplier) {
        double pidRotationVelocity = visionSubsystem.GetRotationVelocityToTarget();
        drivetrain.drive(speedXSupplier.getAsDouble(), speedYSupplier.getAsDouble(), pidRotationVelocity);
    }

    public void Align(DrivetrainSubsystem drivetrain) {
        double pidRotationVelocity = visionSubsystem.GetRotationVelocityToTarget();
        drivetrain.drive(0, 0, pidRotationVelocity);
    }

    public boolean IsOnTarget() {
        return visionSubsystem.IsOnTarget();
    }
    
    public boolean ShootByDistance(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        var isActivelyShooting = false;
        var distanceToTarget = visionSubsystem.GetDistanceToTarget();

        // See if the distance has changed before setting the wheel speeds.
        // A tolerance for variability is needed since the distance can change within inches from the Limelight even though it is standing still.
        if (setDistance == Double.NaN || !MathCommon.WithinTolerance(distanceToTarget, setDistance, VisionConstants.DISTANCE_FROM_TARGET_TOLERANCE_IN_INCHES)) {
            setDistance = distanceToTarget;

            var interpolatedDistance = new InterpolatingDouble(setDistance);
            
            var lowerTargetSpeedMap = ShooterConstants.LOWER_SHOOTER_SPEED_MAP.getInterpolated(interpolatedDistance);
            var upperTargetSpeedMap = ShooterConstants.UPPER_SHOOTER_SPEED_MAP.getInterpolated(interpolatedDistance);

            if (lowerTargetSpeedMap != null && upperTargetSpeedMap != null) {
                lowerTargetSpeed = lowerTargetSpeedMap.value;
                upperTargetSpeed = upperTargetSpeedMap.value;
            }
        }

        if (lowerTargetSpeed != Double.NaN && upperTargetSpeed != Double.NaN) {
            shooter.setSpeeds(lowerTargetSpeed, upperTargetSpeed);
        
            if (IsOnTarget() && shooter.isUpToSpeed()) {
                indexer.feedToShooter();
                isActivelyShooting = true;
            }
        }
        return isActivelyShooting;
    }    

}
