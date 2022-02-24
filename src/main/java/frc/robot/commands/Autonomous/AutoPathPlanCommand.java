// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutoPathPlanCommand {
    
    public static PPSwerveControllerCommand GetCommand(DrivetrainSubsystem drivetrain, PathPlannerTrajectory pathTrajectory) {

        PIDController xController = new PIDController(AutoPathConstants.kPXController, AutoPathConstants.kIXController, AutoPathConstants.kDXController);
        PIDController yController = new PIDController(AutoPathConstants.kPYController, AutoPathConstants.kIYController, AutoPathConstants.kDYController);

        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoPathConstants.kPThetaController, AutoPathConstants.kIThetaController, AutoPathConstants.kDThetaController, AutoPathConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand  swerveControllerCommand = new PPSwerveControllerCommand (
            pathTrajectory,
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
