package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class VisionAlignCommand extends CommandBase {
  DrivetrainSubsystem drivetrain;
  ShooterState shooterState;
  double yDestination = 0;
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

  public double setDistance(){
   double yTarget = Limelight.getTargetY();
   double yDistance = (ShooterConstants.HUB_HEIGHT - ShooterConstants.LIMELIGHT_HEIGHT)/(Math.tan((ShooterConstants.LIMELIGHT_ANGLE + yTarget)));
    if (ShooterConstants.FRONT_OF_HUB_DISTANCE <= yDistance && yDistance >= ShooterConstants.BEHIND_TARMAC_DISTANCE){
      double yFrontOfHub = yDistance - ShooterConstants.FRONT_OF_HUB_DISTANCE;
      double yBehindTarmac = yDistance - ShooterConstants.BEHIND_TARMAC_DISTANCE;
      if (Math.abs(yFrontOfHub) > Math.abs(yBehindTarmac)){
        shooterState = ShooterState.Tarmac;
        yDestination = ShooterConstants.BEHIND_TARMAC_DISTANCE;
      } else if (Math.abs(yFrontOfHub) < Math.abs(yBehindTarmac)){
        shooterState = ShooterState.Hub;
        yDestination = ShooterConstants.FRONT_OF_HUB_DISTANCE;
      }
    } else if (shooterState == ShooterState.Tarmac){
      yDestination = ShooterConstants.BEHIND_TARMAC_DISTANCE;
    } else if (shooterState == ShooterState.LaunchPad){
      yDestination = ShooterConstants.LAUNCH_PAD_DISTANCE;
    } else if (shooterState == ShooterState.ChuckIt){
      yDestination = ShooterConstants.CHUCK_IT_DISTANCE;
    }
    return yDestination;
  }

  @Override
  public boolean isFinished() {
    double xTarget = Limelight.getTargetX();
    double yTarget = Limelight.getTargetY();
    double yDistance = (ShooterConstants.HUB_HEIGHT - ShooterConstants.LIMELIGHT_HEIGHT)/(Math.tan((ShooterConstants.LIMELIGHT_ANGLE + yTarget)));
    double yInbetween = setDistance();
    double yposition = yDistance - yInbetween;
    return Math.abs(xTarget) + Math.abs(yDistance) <= 2;
  }

  private static double getError() {
    return Limelight.getTargetX() + Limelight.getTargetY();
  }

  @Override
  public void execute() {
    double xTarget = Limelight.getTargetX();
    double yTarget = Limelight.getTargetY();
    double yDistance = (ShooterConstants.HUB_HEIGHT - ShooterConstants.LIMELIGHT_HEIGHT)/(Math.tan((ShooterConstants.LIMELIGHT_ANGLE + yTarget)));
    double pidAngularVelocity = pid.calculate(0, -xTarget);
    double yInbetween = setDistance();
    double yposition = yDistance - yInbetween;

    double pidForwardVelocity = pid.calculate(0, yposition);
    drivetrain.drive(pidForwardVelocity, 0, pidAngularVelocity, true);
  }

  public static boolean isAligned() {
    final var error = getError();
    // If it is facing the goal and done rotating
    System.out.println(error);
    return error < 0.1 && error != 0 && DrivetrainSubsystem.getInstance().getAngularVelocity() < 0.5;
  }
}