// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDistanceRotateCommand extends CommandBase {
  
  private DrivetrainSubsystem drivetrain;
  private Pose2d initialPose;
  private Pose2d finalPose;
  private double xDistance;
  private double yDistance;
  private double rotationInDegrees;

  private PIDController pid_X = new PIDController(AutoDriveConstants.kPXController, AutoDriveConstants.kIXController, AutoDriveConstants.kDXController);
  private PIDController pid_Y = new PIDController(AutoDriveConstants.kPYController, AutoDriveConstants.kIYController, AutoDriveConstants.kDYController);

  private ProfiledPIDController rotationController = new ProfiledPIDController(
      AutoDriveConstants.kPThetaController, AutoDriveConstants.kIThetaController, AutoDriveConstants.kIThetaController, AutoDriveConstants.kThetaControllerConstraints);
  
  public AutoDistanceRotateCommand(DrivetrainSubsystem drivetrainSubsystem, double xDistance, double yDistance, double rotationInDegrees) {
    addRequirements(drivetrainSubsystem);
    
    drivetrain = drivetrainSubsystem;

    this.xDistance = xDistance;
    this.yDistance = yDistance;
    this.rotationInDegrees = xDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPose = drivetrain.getPose();

    finalPose = initialPose.plus(new Transform2d(new Translation2d(xDistance, yDistance), Rotation2d.fromDegrees(rotationInDegrees)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final var currentPose = drivetrain.getPose();

    var setVelocityX = pid_X.calculate(currentPose.getX(), finalPose.getX());    
    var setVelocityY = pid_Y.calculate(currentPose.getY(), finalPose.getY());

    final var rotationResult = rotationController.calculate(currentPose.getRotation().getRadians(), finalPose.getRotation().getRadians());

    drivetrain.driveRobotCentric(setVelocityX, setVelocityY, rotationResult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    var done = finalPose.equals(drivetrain.getPose());

    if (!done) {
      var diff = finalPose.minus(drivetrain.getPose());

      done = (Math.abs(diff.getX()) >= xDistance) &&
              (Math.abs(diff.getY()) >= yDistance) &&
              (Math.abs(diff.getRotation().getDegrees()) >= rotationInDegrees);
    }

    return done;
  }
}
