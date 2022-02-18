// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.AutoDriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutoDriveBaseCommand {

    public static SwerveControllerCommand GetCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {

        PIDController xController = new PIDController(AutoDriveConstants.kPXController, AutoDriveConstants.kIXController, AutoDriveConstants.kDXController);
        PIDController yController = new PIDController(AutoDriveConstants.kPYController, AutoDriveConstants.kIYController, AutoDriveConstants.kDYController);

        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoDriveConstants.kPThetaController, 0, 0, AutoDriveConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            DrivetrainConstants.DRIVE_KINEMATICS,
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        return swerveControllerCommand;
    }

}
