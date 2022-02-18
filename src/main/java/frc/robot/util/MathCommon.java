// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class MathCommon {

    public static boolean WithinTolerance(double checkValue, double acceptedTolerance) {
        return Math.abs(checkValue) <= acceptedTolerance;
    }

    public static boolean WithinTolerance(double firstValue, double secondValue, double acceptedTolerance) {
        return WithinTolerance(firstValue - secondValue, acceptedTolerance);
    }
}
