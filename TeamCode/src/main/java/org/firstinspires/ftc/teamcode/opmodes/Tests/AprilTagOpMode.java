package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;


@TeleOp(name = "April Tag Vision Test", group = "Tests")
public class AprilTagOpMode extends LinearOpMode {
    private RobotHardware robot;
    private AprilTagWebcam aprilTagWebcam;

    private static int target_id = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = RobotHardware.getInstance(this);

        aprilTagWebcam = new AprilTagWebcam(robot.aprilTag);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            aprilTagWebcam.telemetryAprilTag(telemetry);

            int hiveTipped = aprilTagWebcam.hiveTipped(target_id);

            telemetry.addData("Hive Tipped: ", hiveTipped);

            telemetry.update();

        }

        if (robot.visionPortal != null) {
            robot.visionPortal.close();
        }
    }
}
