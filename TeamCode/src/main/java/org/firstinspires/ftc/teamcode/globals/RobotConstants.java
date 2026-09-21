package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RobotConstants {

    //TODO: Tune these vals, add more servo positions if needed
    public static double intakePower = 0.55;
    public static double servoUpPos = 0.8;
    public static double servoDownPos = 0.2;

    public static PIDFCoefficients intakePIDFCoefficients = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
}
