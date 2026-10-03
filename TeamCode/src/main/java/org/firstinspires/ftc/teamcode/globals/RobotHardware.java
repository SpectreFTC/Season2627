package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotHardware {

    private static RobotHardware instance;

    private final OpMode opMode;
    public final Motor intakeMotor;
    public final Motor transferMotor;
    public final ServoEx intakeServo;
    public final AprilTagProcessor aprilTag;
    public final VisionPortal visionPortal;
    private RobotHardware(OpMode opMode) {

        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
        transferMotor = new MotorEx(hardwareMap, "transferMotor");
        intakeServo = new ServoEx(hardwareMap, "intakeServo");

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();
    }
    public static RobotHardware getInstance(OpMode opMode) {
        if (opMode == null) {
            return null;
        }
        if (instance == null) {
            instance = new RobotHardware(opMode);
        }
        return instance;
    }
}