package org.firstinspires.ftc.teamcode.globals;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class RobotHardware {

    private static RobotHardware instance;

    private final OpMode opMode;
    public final Motor intakeMotor;
    public final ServoEx intakeServo;

    public RobotHardware(OpMode opMode) {

        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
        intakeServo = new ServoEx(hardwareMap, "intakeServo");
    }


    //copied from off season repo
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