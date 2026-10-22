package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.globals.RobotHardware;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;


public class IntakeSubsystem implements Subsystem {
    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;
    private final Servo flowerServo;
    public IntakeSubsystem(RobotHardware robot) {
        this.intakeMotor = robot.intakeMotor;
        this.transferMotor = robot.transferMotor;
        this.flowerServo =  robot.flowerServo;
    }
    public void setPower(double intakePwr, double transferPwr) {
        intakeMotor.setPower(intakePwr);
        transferMotor.setPower(transferPwr);
    }

    public void intake() {
        setPower(RobotConstants.intakePowerWhileIntaking, RobotConstants.transferPowerWhileIntaking);
    }
    public void hold(){
        setPower(RobotConstants.intakePowerWhileHolding, RobotConstants.transferPowerWhileHolding);
    }
    public void shoot(){
        setPower(RobotConstants.intakePowerWhileShooting, RobotConstants.transferPowerWhileShooting);
    }
    public void eject(){
        setPower(RobotConstants.intakePowerWhileEjecting, RobotConstants.transferPowerWhileEjecting);
    }
    public void flowerIntakeRampDown() {
        flowerServo.setPosition(RobotConstants.flowerServoPosDown);
    }
    public void flowerIntakeRampUp() {
        flowerServo.setPosition(RobotConstants.flowerServoPosUp);
    }


    @Override
    public void update() {

    }

    @Override
    public void init() {

    }
}



