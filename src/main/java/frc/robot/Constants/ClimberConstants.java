// Copy (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ClimberConstants{
    //MOTOR ID
    public static int LOW_CLIMBER_MOTOR_ID = 18;
    public static int HIGH_CLIMBER_MOTOR_ID = 19;

    //MOTOR SPEED
    public static double CLIMBER_MOTOR_SPEED = .5;

    // PNEUMATIC ID
    public static int CLIMBER_HIGH_AIR_IN = 4;
    public static int CLIMBER_HIGH_AIR_OUT = 5;
    public static int CLIMBER_LOW_AIR_OUT = 6;
    public static int CLIMBER_LOW_AIR_IN = 7;

    //MOTOR PIDs
    public static double CLIMBER_MOTOR_P = 0;
    public static double CLIMBER_MOTOR_I = 0;
    public static double CLIMBER_MOTOR_D = 0;
    public static double CLIMBER_MOTOR_F = 0;

    public enum ClimberType {
        Low, High
    }
} 

