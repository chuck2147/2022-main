package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class VisionAlignCommand extends CommandBase {
  DrivetrainSubsystem drivetrain;
  ShooterState shooterState;
  double yDestination = 0;
  double closestPoint = 0;
  ShooterState determineShooterState;
  private PIDController pid = new PIDController(DrivetrainConstants.VISION_ALIGN_P, DrivetrainConstants.VISION_ALIGN_I, DrivetrainConstants.VISION_ALIGN_D, 0.01);

  public VisionAlignCommand(DrivetrainSubsystem drivetrain, ShooterState shooterState) {
    addRequirements(drivetrain);
    // new PIDNTValue(Constants.VISION_ALIGN_P, Constants.VISION_ALIGN_I, Constants.VISION_ALIGN_D, pid, "Vision Align");
    this.drivetrain = drivetrain;
    this.shooterState = shooterState;
  }

  private double howCloseToPoints(double point1, double point2, double yDistance){
   double difference1 = yDistance - point1;
   double difference2 = yDistance - point2;
   if (Math.abs(difference1) >= Math.abs(difference2)) {
     return point2;
   } else if (Math.abs(difference2) >= Math.abs(difference1)){
     return point1;
   }
   return 0;
  }

  private ShooterState determineShooterState(double point1, double point2, double yDistance, ShooterState shooterState1, ShooterState shooterState2){
   double difference1 = yDistance - point1;
   double difference2 = yDistance - point2;
   if (Math.abs(difference1) >= Math.abs(difference2)) {
     return shooterState2;
   } else if (Math.abs(difference2) >= Math.abs(difference1)){
     return shooterState1;
   }
   return shooterState1;
  }
  

  public double setDistance(){
   double yTarget = Limelight.getTargetY();
   double getRadians = Math.toRadians(yTarget);
   double yDistance = (ShooterConstants.HUB_HEIGHT - ShooterConstants.LIMELIGHT_HEIGHT)/(Math.tan((ShooterConstants.LIMELIGHT_ANGLE + getRadians)));
   double yFrontOfHub = ShooterConstants.FRONT_OF_HUB_DISTANCE;
   double yBehindTarmac = ShooterConstants.BEHIND_TARMAC_DISTANCE;
   double yLaunchPad = ShooterConstants.LAUNCH_PAD_DISTANCE;
   double yChuckIt = ShooterConstants.CHUCK_IT_DISTANCE;

    if (ShooterConstants.FRONT_OF_HUB_DISTANCE <= yDistance && yDistance <= ShooterConstants.BEHIND_TARMAC_DISTANCE){
      closestPoint = howCloseToPoints(yFrontOfHub, yBehindTarmac, yDistance);
      determineShooterState = determineShooterState(yFrontOfHub, yBehindTarmac, yDistance, ShooterState.Hub, ShooterState.Tarmac);
    } else if (ShooterConstants.BEHIND_TARMAC_DISTANCE <= yDistance && yDistance <= ShooterConstants.LAUNCH_PAD_DISTANCE){
      closestPoint = howCloseToPoints(yBehindTarmac, yLaunchPad, yDistance);
      determineShooterState = determineShooterState(yBehindTarmac, yLaunchPad, yDistance, ShooterState.Tarmac, ShooterState.LaunchPad);
    } else if (ShooterConstants.LAUNCH_PAD_DISTANCE <= yDistance && yDistance <= ShooterConstants.CHUCK_IT_DISTANCE){
      closestPoint = howCloseToPoints(yLaunchPad, yChuckIt, yDistance);
      determineShooterState = determineShooterState(yLaunchPad, yChuckIt, yDistance, ShooterState.LaunchPad, ShooterState.ChuckIt);
    }
    yDestination = closestPoint;
    return yDestination;
  }

  @Override
  public boolean isFinished() {
    double xTarget = Limelight.getTargetX();
    double yTarget = Limelight.getTargetY();
    double getRadians = Math.toRadians(yTarget);
    double yDistance = (ShooterConstants.HUB_HEIGHT - ShooterConstants.LIMELIGHT_HEIGHT)/(Math.tan((ShooterConstants.LIMELIGHT_ANGLE + getRadians)));
    double yInbetween = setDistance();
    double yposition = yDistance - yInbetween;
    return Math.abs(xTarget) + Math.abs(yposition) <= 2;
  }

  private static double getError() {
    return Limelight.getTargetX() + Limelight.getTargetY();
  }

  @Override
  public void execute() {
    double xTarget = Limelight.getTargetX();
    double yTarget = Limelight.getTargetY();
    double getRadians = Math.toRadians(yTarget);
    double yDistance = (ShooterConstants.HUB_HEIGHT - ShooterConstants.LIMELIGHT_HEIGHT)/(Math.tan((ShooterConstants.LIMELIGHT_ANGLE + getRadians)));
    double pidAngularVelocity = pid.calculate(0, -xTarget);
    double yInbetween = setDistance();
    double yposition = yDistance - yInbetween;

    double pidForwardVelocity = pid.calculate(0, yposition);
    drivetrain.drive(new ChassisSpeeds(pidForwardVelocity, 0, pidAngularVelocity));
  }

  public static boolean isAligned() {
    final var error = getError();
    // If it is facing the goal and done rotating
    System.out.println(error);
    return error < 0.1 && error != 0 && DrivetrainSubsystem.getInstance().getAngularVelocity() < 0.5;
  }
}
