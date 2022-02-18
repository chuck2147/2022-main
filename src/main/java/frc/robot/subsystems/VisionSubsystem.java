// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.vision.Limelight;

public class VisionSubsystem extends SubsystemBase {

  public static final PIDController visionPID = new PIDController(VisionConstants.VISION_ALIGN_P, VisionConstants.VISION_ALIGN_I, VisionConstants.VISION_ALIGN_D, 0.01);

  ShuffleboardTab tab = Shuffleboard.getTab("NTValues");
  NetworkTableEntry distanceToTarget = tab.add("Distance To Target", 0)
    .withSize(2, 1)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  public double GetRotationVelocityToTarget() {
    var velocity = 0.0;
    var setPoint = Double.NaN;

    if (Limelight.hasTarget()) {
      if (!IsOnTarget()) {
        setPoint = -GetHorizontalOffset();
      }
    }
    else {
      setPoint = 306.0;
    }

    if (setPoint != Double.NaN) {
      velocity = visionPID.calculate(0, setPoint);
    }

    return velocity;
  }

  public boolean IsOnTarget() {
    double horizontalOffset = GetHorizontalOffset();

    return Math.abs(horizontalOffset) <= VisionConstants.HORIZONTAL_TOLERANCE_IN_DEGREES;
  }
  
  public double GetDistanceToTarget() {
    double getRadians = Math.toRadians(GetVerticalOffset());
    double yDistance = (VisionConstants.HUB_HEIGHT - VisionConstants.LIMELIGHT_HEIGHT)/(Math.tan((VisionConstants.LIMELIGHT_ANGLE + getRadians)));

    return yDistance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distanceToTarget.setValue(GetDistanceToTarget());

  }

  private double GetHorizontalOffset() {
    return Limelight.getTargetX();
  }

  private double GetVerticalOffset() {
    return Limelight.getTargetY(); 
  }

}
