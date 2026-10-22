package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //intake
    public static double intakePowerWhileIntaking;
    public static double transferPowerWhileIntaking;
    public static double intakePowerWhileEjecting;
    public static double transferPowerWhileEjecting;
    public static double intakePowerWhileHolding;
    public static double transferPowerWhileHolding;
    public static double intakePowerWhileShooting;
    public static double transferPowerWhileShooting;
    public static double flowerServoPosDown;
    public static double flowerServoPosUp;
    //shooter
    public static double kV = 0.005134;
    public static double kS = 1.1;
    public static double SHOOTER_kP = 0.015;
    public static double SHOOTER_kI = 0;
    public static double SHOOTER_kD = 0;
    public static double SHOOTER_I_ZONE;
    public static double SHOOTER_MAX_INTEGRAL_VOLTS;
    public static double mVelocity;
    public static double bVelocity;
    public static double mHoodAngle;
    public static double bHoodAngle;
    public static double hoodServoRange;
    public static double hoodServoPositionAtCalibration;
    public static double hoodServoAngleAtCalibration;
    public static double SHOOTER_DECAY_FACTOR = 1;
}
