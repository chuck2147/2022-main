package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    MotorController leftMotor = new WPI_VictorSPX(DrivetrainConstants.DRIVETRAIN_LEFT_MOTOR_ID);
    MotorController rightMotor = new WPI_VictorSPX(DrivetrainConstants.DRIVETRAIN_RIGHT_MOTOR_ID);

    private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);


    public DrivetrainSubsystem () {

    }

    public void tankDrive(double rightSpeed, double leftSpeed) {
        diffDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        diffDrive.arcadeDrive(xSpeed*DrivetrainConstants.DRIVE_SPEED_SCALE, zRotation*DrivetrainConstants.DRIVE_SPEED_SCALE);
    }

}
