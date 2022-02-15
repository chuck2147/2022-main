// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.vision.Limelight;

public class VisionSubsystem extends SubsystemBase {

  public static final PIDController visionPID = new PIDController(VisionConstants.VISION_ALIGN_P, VisionConstants.VISION_ALIGN_I, VisionConstants.VISION_ALIGN_D, 0.01);
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  public double GetRotationVelocityToTarget() {
    return visionPID.calculate(0, -GetHorizontalOffset());
  }

  public boolean IsOnTarget() {
    double horizontalOffset = GetHorizontalOffset();

    return Math.abs(horizontalOffset) <= VisionConstants.HORIZONTAL_TOLERANCE_IN_DEGREES;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  private double GetHorizontalOffset() {
    return Limelight.getTargetX();
  }
}
