package frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Controller;


public class ControllerConstants {
    private final static Controller driverController = new Controller(0, 0.05);
    private final static Controller operatorController = new Controller(1, 0.05);
    //CLIMBER BUTTONS
    public static JoystickButton CLIMB_LOW_UP_BUTTON = driverController.getButton(Controller.Button.A);
    public static JoystickButton CLIMB_LOW_DOWN_BUTTON = driverController.getButton(Controller.Button.B);
    public static JoystickButton CLIMB_HIGH_UP_BUTTON = driverController.getButton(Controller.Button.X);
    public static JoystickButton CLIMB_HIGH_DOWN_BUTTON = driverController.getButton(Controller.Button.Y);
    //INTAKE BUTTONS
    public static JoystickButton INTAKE_IN_BUTTON = driverController.getButton(Controller.Button.RightBumper);
    public static JoystickButton INTAKE_OUT_BUTTON = driverController.getButton(Controller.Button.LeftBumper);
    //SHOOTER BUTTONS
    public static JoystickButton FRONT_OF_HUB_SHOT = operatorController.getButton(Controller.Button.A);
    public static JoystickButton BEHIND_TARMAC_SHOT = operatorController.getButton(Controller.Button.B);
    public static JoystickButton LAUNCH_PAD_SHOT = operatorController.getButton(Controller.Button.X);
    public static JoystickButton CHUCK_IT_SHOT = operatorController.getButton(Controller.Button.Y);
    //INDEXER BUTTONS
    public static JoystickButton INDEX_IN = operatorController.getButton(Controller.Button.RightBumper);
    public static JoystickButton INDEX_OUT = operatorController.getButton(Controller.Button.LeftBumper);
    //MISC BUTTONS
    public static JoystickButton RESET_GYRO = driverController.getButton(Controller.Button.Start);
}
