package frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AxisTrigger;
import frc.robot.Controller;
import frc.robot.subsystems.DrivetrainSubsystem;


public class ControllerConstants {

    private final static Controller driverController = new Controller(0, 0.05);
    private final static Controller operatorController = new Controller(1, 0.05);

    //---------------------------
    // Driver Controller
    //DRIVER JOYSTICKS
    public static DoubleSupplier DRIVE_Y = () -> -modifyAxis(driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    public static DoubleSupplier DRIVE_X =  () -> -modifyAxis(driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    public static DoubleSupplier DRIVE_ROTATION = () -> -modifyAxis(driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    //SHOOTER BUTTONS
    public static JoystickButton FRONT_OF_HUB_SHOT = driverController.getButton(Controller.Button.A);
    public static JoystickButton BEHIND_TARMAC_SHOT = driverController.getButton(Controller.Button.X);
    public static JoystickButton LAUNCH_PAD_SHOT = driverController.getButton(Controller.Button.Y);
    public static JoystickButton CHUCK_IT_SHOT = driverController.getButton(Controller.Button.RightBumper);
    public static JoystickButton VISION_ALIGN = driverController.getButton(Controller.Button.B);
    public static JoystickButton VISION_AND_SHOOTER_ALIGN = driverController.getButton(Controller.Button.LeftBumper);

    //MISC BUTTONS
    public static JoystickButton RESET_GYRO = driverController.getButton(Controller.Button.Start);
    public static JoystickButton RESET_POSE = driverController.getButton(Controller.Button.Back);

    //-------------------------
    // Operator Controller

    //CLIMBER BUTTONS
    public static DoubleSupplier CLIMB_RIGHT_AXIS = () -> -operatorController.getRightY();
    public static DoubleSupplier CLIMB_LEFT_AXIS = () -> -operatorController.getLeftY();
    //INTAKE BUTTONS
    public static AxisTrigger RUN_INTAKE_REVERSE = new AxisTrigger(operatorController, 2);
    public static AxisTrigger RUN_COLLECT_INTAKE = new AxisTrigger(operatorController, 3);
    public static JoystickButton STOP_INTAKE = operatorController.getButton(Controller.Button.A);
    
    //INDEXER BUTTONS
    public static JoystickButton INDEX_IN = operatorController.getButton(Controller.Button.RightBumper);
    public static JoystickButton INDEX_OUT = operatorController.getButton(Controller.Button.LeftBumper);
    
    private static double deadband(double value, double deadband) {
      if (Math.abs(value) > deadband) {
        if (value > 0.0) {
          return (value - deadband) / (1.0 - deadband);
        } else {
          return (value + deadband) / (1.0 - deadband);
        }
      } else {
        return 0.0;
      }
    }
  
    private static double modifyAxis(double value) {
      // Deadband
      value = deadband(value, 0.1);
  
      // Square the axis
      value = Math.copySign(value * value, value);
  
      return value;
    }
}
