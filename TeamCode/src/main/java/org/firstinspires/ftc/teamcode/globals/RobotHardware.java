package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotHardware {

    private static RobotHardware instance;

    private final OpMode opMode;
    public final Servo flowerServo, hoodServo;
    public final DcMotorEx intakeMotor, transferMotor, shooterMotor1, shooterMotor2;
    public final DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public final GoBildaPinpointDriver odo;
    public final VoltageSensor voltageSensor;

    private RobotHardware(OpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        leftFront = hardwareMap.get(DcMotorEx.class, "cm1");
        rightFront = hardwareMap.get(DcMotorEx.class, "cm3");
        leftBack = hardwareMap.get(DcMotorEx.class, "cm0");
        rightBack = hardwareMap.get(DcMotorEx.class, "cm2");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "em0");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "em1");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "em2");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor = hardwareMap.get(DcMotorEx.class, "em3");
        flowerServo = hardwareMap.get(Servo.class, "cs1");
        hoodServo = hardwareMap.get(Servo.class, "cs0");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "ei0");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    //copied from off season repo
    public static RobotHardware getInstance(OpMode opMode) {
        if (opMode == null) {
            return null;
        }
        // Rebuild whenever a new OpMode asks, so devices are re-fetched and motor
        // directions are re-applied after the SDK resets them between runs
        if (instance == null || instance.opMode != opMode) {
            instance = new RobotHardware(opMode);
        }
        return instance;
    }
}