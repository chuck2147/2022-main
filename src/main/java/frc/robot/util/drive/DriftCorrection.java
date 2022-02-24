// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class DriftCorrection {
    double pid_P = 0.07;
    double pid_I = 0.0;
    double pid_D = 0.004;

    PIDController driftCorrectionPID = new PIDController(pid_P, pid_I, pid_D);
    double desiredHeading;
    double previousXY = 0;

    public ChassisSpeeds correctSpeeds(ChassisSpeeds speeds, Pose2d pose) {
        var retSpeeds = speeds;

        double xy = Math.abs(retSpeeds.vxMetersPerSecond) + Math.abs(retSpeeds.vyMetersPerSecond);

        if (Math.abs(retSpeeds.omegaRadiansPerSecond) > 0.0 || previousXY <= 0) {
            desiredHeading = pose.getRotation().getDegrees();
        }
        else if (xy > 0) {
            retSpeeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(pose.getRotation().getDegrees(), desiredHeading);
        }

        previousXY = xy;

        return retSpeeds;
    }
}
