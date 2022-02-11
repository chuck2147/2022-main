package frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AxisTrigger;
import frc.robot.Controller;


public class ControllerConstants {

    private final static Controller driverController = new Controller(0, 0.05);
    private final static Controller operatorController = new Controller(1, 0.05);

    //---------------------------
    // Driver Controller

    //SHOOTER BUTTONS
    public static JoystickButton FRONT_OF_HUB_SHOT = driverController.getButton(Controller.Button.A);
    public static JoystickButton BEHIND_TARMAC_SHOT = driverController.getButton(Controller.Button.X);
    public static JoystickButton LAUNCH_PAD_SHOT = driverController.getButton(Controller.Button.Y);
    //public static JoystickButton CHUCK_IT_SHOT = driverController.getButton(Controller.Button.B);

    //MISC BUTTONS
    public static JoystickButton RESET_GYRO = driverController.getButton(Controller.Button.Start);

    //-------------------------
    // Operator Controller

    //CLIMBER BUTTONS
    public static DoubleSupplier CLIMB_RIGHT_AXIS = () -> operatorController.getRightY();
    public static DoubleSupplier CLIMB_LEFT_AXIS = () -> operatorController.getLeftY();
    //INTAKE BUTTONS
    public static AxisTrigger RUN_INTAKE_REVERSE = new AxisTrigger(operatorController, 2);
    
    //INDEXER BUTTONS
    public static JoystickButton INDEX_IN = operatorController.getButton(Controller.Button.RightBumper);
    public static JoystickButton INDEX_OUT = operatorController.getButton(Controller.Button.LeftBumper);
    
}
