package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.globals.RobotHardware;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;


public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware robot;

    private PIDFController intakeController;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;

        robot.intakeMotor.setInverted(false);
        robot.transferMotor.setInverted(false);
        robot.intakeServo.setInverted(false);

        robot.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intakeController = new PIDFController(RobotConstants.intakePIDFCoefficients);
    }

    public void setIntakePower(double power) {
        robot.intakeMotor.set(power);
    }
    public void setTransferPower(double power) {
        robot.transferMotor.set(power);
    }

    public void intake() {
        setIntakePower(RobotConstants.intakePower);
        setTransferPower(-RobotConstants.intakePower);
    }

    public void rampUp(){
        robot.intakeServo.set(RobotConstants.servoUpPos);
    }

    public void rampDown() {
        robot.intakeServo.set(RobotConstants.servoDownPos);
    }
    public void setVelocity(double setpoint) {
        robot.intakeMotor.set(intakeController.calculate(setpoint));
        //Method must be called each cycle (.calculate() needs to be called each cycle)
    }

}



