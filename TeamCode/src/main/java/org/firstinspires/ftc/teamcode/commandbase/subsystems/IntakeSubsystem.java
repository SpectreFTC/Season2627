package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.globals.RobotHardware;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class IntakeSubsystem extends SubsystemBase implements RobotConstants {
    private final RobotHardware robot;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;

        robot.intakeMotor.setInverted(false);
        robot.intakeServo.setInverted(false);

        robot.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        robot.intakeMotor.set(power);
    }

    public void intake() {
        setPower(intakePower);
    }

    public void servoUp() {
        robot.intakeServo.set(servoUpPos);
    }

    public void servoDown() {
        robot.intakeServo.set(servoDownPos);
    }

}



