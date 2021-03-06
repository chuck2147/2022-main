package frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AxisTrigger;
import frc.robot.Controller;


public class ControllerConstants {

    private final static Controller driverController = new Controller(0, 0.05);
    private final static Controller operatorController = new Controller(1, 0.05);

    private final static double DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;
    //private final static double DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
    

    //---------------------------
    // Driver Controller
    //DRIVER JOYSTICKS
    public static DoubleSupplier DRIVE_Y = () -> -modifyAxis(driverController.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    public static DoubleSupplier DRIVE_X =  () -> -modifyAxis(driverController.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    public static DoubleSupplier DRIVE_ROTATION = () -> -modifyAxis(driverController.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    //SHOOTER BUTTONS
    public static JoystickButton FRONT_OF_HUB_SHOT = driverController.getButton(Controller.Button.A);
    public static JoystickButton BEHIND_TARMAC_SHOT = driverController.getButton(Controller.Button.X);
    public static JoystickButton LAUNCH_PAD_SHOT = driverController.getButton(Controller.Button.Y);
    //public static JoystickButton CHUCK_IT_SHOT = driverController.getButton(Controller.Button.RightBumper);
    public static JoystickButton VISION_ALIGN = driverController.getButton(Controller.Button.B);
    public static JoystickButton VISION_AND_SHOOTER_ALIGN = driverController.getButton(Controller.Button.LeftBumper);

    //MISC BUTTONS
    public static JoystickButton RESET_GYRO = driverController.getButton(Controller.Button.Start);
    public static JoystickButton RESET_POSE = driverController.getButton(Controller.Button.Back);

    //-------------------------
    // Operator Controller

    //CLIMBER BUTTONS
    public static DoubleSupplier CLIMB_LOW_AXIS = () -> operatorController.getRightY();
    public static DoubleSupplier CLIMB_HIGH_AXIS = () -> -operatorController.getLeftY();
    public static JoystickButton HOOK_PISTON = driverController.getButton(Controller.Button.RightBumper);
    //INTAKE BUTTONS
    public static AxisTrigger RUN_INTAKE_REVERSE = new AxisTrigger(operatorController, 2);
    public static AxisTrigger RUN_COLLECT_INTAKE = new AxisTrigger(operatorController, 3);
    
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

      //value = rateLimit(value);
  
      return value;
    }

    private static double rateLimit(double speed) {
      var rateLimiter = new SlewRateLimiter(DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
      return rateLimiter.calculate(speed);
    }
}
