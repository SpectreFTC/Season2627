package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.concurrent.TransferQueue;

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
    public static double transferPowerWhileShooting = -0.7;
    public static double flowerServoPosDown = 0.5;
    public static double flowerServoPosUp = 1;
    //shooter
    public static double kV = 0.005134;
    public static double kS = 1.1;
    public static double SHOOTER_kP = 0.078;
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
    public static double setVelocity = 725;
    public static double threshold = 50;
    public static double setHoodAngle = 22;

    //april tag

    public static Position cameraPose = new Position(DistanceUnit.MM, 35.292, -109.931, 301.822, 0);
    public static YawPitchRollAngles cameraAngle = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 32.337, 0, 0);
}
