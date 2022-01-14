package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller extends XboxController {
  Controller(int port, Hand hand, double deadband) {
    super(port);
    this.deadband = deadband;
  }
  double deadband;

  public enum Button {
    A(1), B(2), X(3), Y(4), LeftBumper(5), RightBumper(6), Back(7), Start(8);
    Button(int port) {
      this.port = port;
    }
    final int port;
  }

  public JoystickButton getButton(Button button) {
    return new JoystickButton(this, button.port);
  }

  public enum Hand {
    Left, Right
  }
  
  public double getX(Hand hand) {
    var ret = 0.0;
    switch(hand) {
      case Left:
        ret = applyDeadband(super.getLeftY(), deadband);
        break;
      case Right:
        ret = applyDeadband(super.getRightY(), deadband);
        break;
    }
    return ret;
  }

  public double getY(Hand hand) {
      var ret = 0.0;
      switch(hand) {
        case Left:
          ret = applyDeadband(super.getLeftY(), deadband);
          break;
        case Right:
          ret = applyDeadband(super.getRightY(), deadband);
          break;
      }
      return ret;
  }
  
  public double getRawAxis(int axis) {
    return applyDeadband(super.getRawAxis(axis), deadband);
  }

  protected double applyDeadband(double value, double deadband) {
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
}
