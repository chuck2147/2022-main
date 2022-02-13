// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignShooterSideCommand extends CommandBase {
  DrivetrainSubsystem drivetrain;
  DoubleSupplier speedXSupplier;
  DoubleSupplier speedYSupplier;
  private PIDController pid = new PIDController(MiscConstants.VISION_ALIGN_P, MiscConstants.VISION_ALIGN_I, MiscConstants.VISION_ALIGN_D, 0.01);
  
  public AlignShooterSideCommand(DrivetrainSubsystem drivetrain, DoubleSupplier speedXSupplier, DoubleSupplier speedYSupplier) {
    addRequirements(drivetrain);
    // new PIDNTValue(Constants.VISION_ALIGN_P, Constants.VISION_ALIGN_I, Constants.VISION_ALIGN_D, pid, "Vision Align");
    this.drivetrain = drivetrain;
    this.speedXSupplier = speedXSupplier;
    this.speedYSupplier = speedYSupplier;
  }
  
  @Override
  public boolean isFinished() {
    double xTarget = Limelight.getTargetX();
    return Math.abs(xTarget) <= 1;
  }

  // private static double getError() {
  //   return Limelight.getTargetX();
  // }

  @Override
  public void execute() {
    double xTarget = Limelight.getTargetX();
    double pidAngularVelocity = pid.calculate(0, -xTarget);
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedXSupplier.getAsDouble(), speedYSupplier.getAsDouble(), pidAngularVelocity, drivetrain.getGyroscopeRotation()));
    //drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, pidAngularVelocity, drivetrain.getGyroscopeRotation()));
  }

  // public static boolean isAligned() {
  //   final var error = getError();
  //   // If it is facing the goal and done rotating
  //   System.out.println(error);
  //   return error < 0.1 && error != 0 && DrivetrainSubsystem.getInstance().getAngularVelocity() < 0.5;
  // }
}
