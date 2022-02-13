package frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;

public class MiscConstants {
    //LIMELIGHT PIDs 
    public static final double VISION_ALIGN_P = 0.185;
    public static final double VISION_ALIGN_I = 0;
    public static final double VISION_ALIGN_D = 0.005;
    public static final double VISION_ALIGN_F = 0;

    public static final PIDController VISION_PID = new PIDController(MiscConstants.VISION_ALIGN_P, MiscConstants.VISION_ALIGN_I, MiscConstants.VISION_ALIGN_D, 0.01);
    //LIMELIGHT IP: 169.254.66.52
}
