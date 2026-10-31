package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //intake
    public static double intakePowerWhileIntaking = 1;
    public static double transferPowerWhileIntaking = 1;
    public static double intakePowerWhileEjecting = -1;
    public static double transferPowerWhileEjecting = 1;
    public static double intakePowerWhileHolding = 0.2;
    public static double transferPowerWhileHolding = 0.1;
    public static double intakePowerWhileShooting = 1;
    public static double transferPowerWhileShooting = -0.9;
    public static double flowerServoPosDown = 0.41;
    public static double flowerServoPosUp = 0.95;
    //shooter
    public static double kV = 0.005134;
    public static double kS = 1.15;
    public static double SHOOTER_kP = 0.07565;
    public static double SHOOTER_kI = 0;
    public static double SHOOTER_kD = 0;
    public static double SHOOTER_I_ZONE;
    public static double SHOOTER_MAX_INTEGRAL_VOLTS;
    public static double mVelocity = 0.645201;
    public static double bVelocity = 688.687;
    public static double mHoodAngle = 0.0365514;
    public static double bHoodAngle = 15.8227;
    public static double hoodServoRange = 25.760525 - 5.252703;
    public static double hoodServoPositionAtCalibration = 0.16;
    public static double hoodServoAngleAtCalibration = 5.252703;
    public static double SHOOTER_DECAY_FACTOR = 1;
    public static double maxServoPos = 0.89;
    public static double setVelocity = 700;
    public static double autonSetVelocityRed = 690;
    public static double autonSetVelocityBlue = 690;
    public static double setHoodAngle = 19;
    public static double AIM_KP = 1;
}
