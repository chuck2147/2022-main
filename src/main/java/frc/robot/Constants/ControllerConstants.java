package frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AxisTrigger;
import frc.robot.Controller;


public class ControllerConstants {
    private final static Controller driverController = new Controller(0, 0.05);
    private final static Controller operatorController = new Controller(1, 0.05);
    //CLIMBER BUTTONS
    public static JoystickButton CLIMB_RIGHT_UP_BUTTON = driverController.getButton(Controller.Button.RightBumper);
    public static AxisTrigger CLIMB_RIGHT_DOWN_BUTTON = new AxisTrigger(driverController, 2);
    public static JoystickButton CLIMB_LEFT_UP_BUTTON = driverController.getButton(Controller.Button.LeftBumper);
    public static AxisTrigger CLIMB_LEFT_DOWN_BUTTON = new AxisTrigger(driverController, 3);
    //INTAKE BUTTONS
    public static AxisTrigger INTAKE_IN_BUTTON = new AxisTrigger(operatorController, 2);
    public static AxisTrigger INTAKE_OUT_BUTTON = new AxisTrigger(operatorController, 3);
    //SHOOTER BUTTONS
    public static JoystickButton FRONT_OF_HUB_SHOT = driverController.getButton(Controller.Button.A);
    public static JoystickButton BEHIND_TARMAC_SHOT = driverController.getButton(Controller.Button.X);
    public static JoystickButton LAUNCH_PAD_SHOT = driverController.getButton(Controller.Button.Y);
    //public static JoystickButton CHUCK_IT_SHOT = driverController.getButton(Controller.Button.B);
    //INDEXER BUTTONS
    public static JoystickButton INDEX_IN = operatorController.getButton(Controller.Button.RightBumper);
    public static JoystickButton INDEX_OUT = operatorController.getButton(Controller.Button.LeftBumper);
    //MISC BUTTONS
    public static JoystickButton RESET_GYRO = driverController.getButton(Controller.Button.Start);
}
