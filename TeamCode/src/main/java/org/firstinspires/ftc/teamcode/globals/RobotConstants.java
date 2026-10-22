package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //intake
    public static double intakePowerWhileIntaking = 1;
    public static double transferPowerWhileIntaking = 0.3;
    public static double intakePowerWhileEjecting = -1;
    public static double transferPowerWhileEjecting = 1;
    public static double intakePowerWhileHolding = 0.2;
    public static double transferPowerWhileHolding = 0.1;
    public static double intakePowerWhileShooting = 1;
    public static double transferPowerWhileShooting = -1;
    public static double flowerServoPosDown = 0.5;
    public static double flowerServoPosUp = 1;
    //shooter
    public static double kV = 0.005134;
    public static double kS = 1.1;
    public static double SHOOTER_kP = 0.089;
    public static double SHOOTER_kI = 0;
    public static double SHOOTER_kD = 0;
    public static double SHOOTER_I_ZONE;
    public static double SHOOTER_MAX_INTEGRAL_VOLTS;
    public static double mVelocity = 1.67224;
    public static double bVelocity = 677.542;
    public static double mHoodAngle = 0.342809;
    public static double bHoodAngle = 6.02107;
    public static double hoodServoRange = 25.760525 - 5.252703;
    public static double hoodServoPositionAtCalibration = 0.16;
    public static double hoodServoAngleAtCalibration = 5.252703;
    public static double SHOOTER_DECAY_FACTOR = 1;
}
