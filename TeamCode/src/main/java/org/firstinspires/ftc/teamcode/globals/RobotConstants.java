package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.geometry.Pose2d;

public class RobotConstants {

    public enum HiveState { NOT_VISIBLE, LEVEL, UP, DOWN }

    //TODO: Tune these vals, add more servo positions if needed
    public static double intakePower = 0.55;
    public static double servoUpPos = 0.8;
    public static double servoDownPos = 0.2;
    public static double hiveHeightUp = 51.0;
    public static double hiveHeightDown = 37.0;
    public static double cameraHeightIn = 9.86;
    public static double cameraAngle = 32.37;
    public static double cameraForwardOffsetIn = -6.32;
    public static double cameraLeftOffsetIn = 0.0;

    public static PIDFCoefficients intakePIDFCoefficients = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);


}
