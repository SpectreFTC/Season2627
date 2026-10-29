package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.globals.RobotHardware;

public class MecanumDriveSubsystem implements Subsystem {
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;


    public MecanumDriveSubsystem(RobotHardware robotHardware) {
        this.leftFront = robotHardware.leftFront;
        this.leftBack = robotHardware.leftBack;
        this.rightFront = robotHardware.rightFront;
        this.rightBack = robotHardware.rightBack;
    }


    /**
     * Drives the robot in a field-centric control.
     * The X/Y power inputs are given in a fixed world (field) coordinate system defined by the user.
     * These inputs are rotated into the robot's local coordinate system using the current heading
     * (from odometry), so the robot moves consistently relative to the field regardless of its orientation.
     *
     * @param xPower     - Desired power around the field's x-axis (axial)
     * @param yPower     - Desired power around the field's y-axis (lateral)
     * @param turnPower  - not needed to change for the robot (yaw)
     * @param odoHeading - current odometry heading information in radians (odoHeading)
     */
    public void driveFieldCentric(double xPower, double yPower, double turnPower, double odoHeading) {
        Translation2d power = new Translation2d(xPower, yPower);
        double strafe = power.rotateBy(new Rotation2d(odoHeading)).getY();
        double forward = power.rotateBy(new Rotation2d(odoHeading)).getX();
        driveRobotCentric(forward, strafe, turnPower);
    }


    /**
     * Drives the robot in a robot-centric manner (relative to the robot's own orientation).
     * <p>By convention, positive lateral and positive yaw inputs are inverted so that:</p>
     * <li>Positive lateral moves the robot to the right.</li>
     * <li>Positive yaw turns the robot to the right.</li>
     * <p>
     *
     * @param axial   Forward/backward power. Positive values move forward.
     * @param lateral Strafe power. Positive values move right.
     * @param yaw     Turn power. Positive values turn right.
     */

    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double frontLeftPower = (axial + lateral + yaw) / denominator;
        double backLeftPower = (axial - lateral + yaw) / denominator;
        double frontRightPower = (axial - lateral - yaw) / denominator;
        double backRightPower = (axial + lateral - yaw) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    /**
     * Turns off the drivetrain motors by setting their power to 0.
     */
    public void turnOffMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }

    @Override
    public void update() {

    }

    @Override
    public void init() {

    }
}
