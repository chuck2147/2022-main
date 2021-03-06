// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.drive.DriftCorrection;

public class DrivetrainSubsystem extends SubsystemBase {
  private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

  public static DrivetrainSubsystem getInstance() {
    return instance;
  }
  /**
   * The maximum volta
   * ge that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;  

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  private final PigeonIMU m_pigeon = new PigeonIMU(DrivetrainConstants.DRIVETRAIN_PIGEON_ID);
  private Pose2d m_pose = new Pose2d(0, 0, new Rotation2d());
  private final double SCALE_X = 1; // -1/0.9;
  private final double SCALE_Y = 1; // -1/0.9;

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable currentPoseTable = nt.getTable("/pathFollowing/current");
  private final NetworkTableEntry currentXEntry = currentPoseTable.getEntry("x");
  private final NetworkTableEntry currentYEntry = currentPoseTable.getEntry("y");
  private final NetworkTableEntry currentAngleEntry = currentPoseTable.getEntry("angle");  
  private final NetworkTableEntry currentGyro = currentPoseTable.getEntry("gyro");

  private final Field2d field = new Field2d();

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DrivetrainConstants.DRIVE_KINEMATICS, 
    new Rotation2d(0),
    //Starting Postition and Angle
    m_pose);


  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  // For correcting drift.
  DriftCorrection driftCorrection = new DriftCorrection();

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    SmartDashboard.putData("field", field);
    
    resetGyroscope();
    resetPose(new Vector2d(0,0), new Rotation2d(0));

    // Run resetPost on another Thread and delay for 1 second so Gyroscope is done calibrating on startup.
    // new Thread(() -> {
    //   try {
    //     Thread.sleep(1000);
    //     resetGyroscope();
    //     resetPose(new Vector2d(0,0), new Rotation2d(0));
    //   }
    //   catch (Exception ex) {
    //     // System.out.println(ex.toString());
    //   }
    // }).start();
    
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.

    GearRatio swerveGearRatio = Mk4SwerveModuleHelper.GearRatio.L2;

    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            swerveGearRatio,
            // This is the ID of the drive motor
            DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            swerveGearRatio,
            DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            swerveGearRatio,
            DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
            DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            swerveGearRatio,
            DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
            DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET
    );
    
    SetStateFromSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  /**
   * Get the current pose in meters.
   * @return
   */
  public Pose2d getPose() {
        return m_pose;
  }

  public Pose2d getScaledPose() {
     final var translation = new Translation2d(m_pose.getX() * SCALE_X, m_pose.getY() * SCALE_Y);
     final var rotation = m_pose.getRotation().rotateBy(new Rotation2d(0));
    
     return new Pose2d(translation.getX(), translation.getY(), rotation);
  }

  public void resetOdometry(Pose2d pose) {
    //resetGyroscope(pose.getRotation().getDegrees());

    var vector = new Vector2d(pose.getX(), pose.getY());
    resetPose(vector, pose.getRotation());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public double getGyroDegrees() {
    var gyroDegrees = m_pigeon.getFusedHeading();
    //var gyroDegrees = m_pigeon.getYaw();
    return gyroDegrees;
    //return Math.IEEEremainder(gyroDegrees, 360);
  }
  
  public void resetGyroscope() {
    resetGyroscope(0.0);
  }
  
  
  public void resetGyroscope(double angle) {
    // Use Degrees
    // var heading = angle * (23040 / 360); // Thjs maybe needed to use to set to an actual angle.
    // m_pigeon.setFusedHeading(heading);
    m_pigeon.setFusedHeading(angle);

    //m_pigeon.setYaw(angle);    

    System.out.println("Resesting Gyroscope to " + angle);  
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(getGyroDegrees());
  }

  public double getAngularVelocity() {
    double[] angularVelocities = new double[3];
    m_pigeon.getRawGyro(angularVelocities); 
    return angularVelocities[2];
  }

  private void updatePoseNT() {
    final var pose = getScaledPose();

    currentGyro.setDouble(getGyroscopeRotation().getDegrees());
    currentAngleEntry.setDouble(pose.getRotation().getDegrees());
    currentXEntry.setDouble(pose.getX());
    currentYEntry.setDouble(pose.getY());

  }

  public void resetPose(Vector2d translation, Rotation2d angle) {
    System.out.println("Reset Pose");
    m_odometry.resetPosition(
      new Pose2d(
        //coordinates switched x is forward, y is left and right.
        // Converting to unit system of path following which uses x for right and left
        new Translation2d(translation.x / SCALE_X, translation.y / SCALE_Y),
        angle
      ),
      getGyroscopeRotation()
    );
    m_pose = m_odometry.getPoseMeters();
    updatePoseNT();
  }

  public void drive(double xDriveSpeed, double yDriveSpeed, double rotationSpeed) {
    var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xDriveSpeed,
      yDriveSpeed,
      rotationSpeed,
      getGyroscopeRotation()
    );

    chassisSpeeds = driftCorrection.correctSpeeds(chassisSpeeds, getPose());

    SetStateFromSpeeds(chassisSpeeds);
  }

  public void driveRobotCentric(double xDriveSpeed, double yDriveSpeed, double rotationSpeed) {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    var chassisSpeeds = new ChassisSpeeds(
      xDriveSpeed,
      yDriveSpeed,
      rotationSpeed
    );

    SetStateFromSpeeds(chassisSpeeds);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    
    m_pose = m_odometry.update(getGyroscopeRotation(), states[0], states[1], states[2], states[3]);
  }


  @Override
  public void periodic() {
    updatePoseNT();

    // Also update the Field2D object (so that we can visualize this in sim)
    field.setRobotPose(getPose());
  }

  private void SetStateFromSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  
}
