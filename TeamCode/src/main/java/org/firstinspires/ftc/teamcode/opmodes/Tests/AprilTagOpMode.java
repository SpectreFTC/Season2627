package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.PinpointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.vision.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;
import org.firstinspires.ftc.teamcode.globals.TagPoses;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@TeleOp(name = "April Tag Vision Test", group = "Tests")
public class AprilTagOpMode extends LinearOpMode {
    private RobotHardware robot;
    private AprilTagWebcam aprilTagWebcam;
    MultipleTelemetry multipleTelemetry;
    Follower follower;
    CommandScheduler scheduler;
    private static int target_id = 40;
    private Pose2d start = new Pose2d(70.5, 70.5, Math.toRadians(90));
    private Pose followerStart = new Pose(70.5, 70.5, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = RobotHardware.getInstance(this);

        aprilTagWebcam = new AprilTagWebcam(robot.aprilTagProcessor);
        PinpointOdometrySubsystem odo = new PinpointOdometrySubsystem(robot, start);
        follower = Constants.create(hardwareMap);
        follower.setPose(followerStart);
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        scheduler = CommandScheduler.getInstance();
        scheduler.reset();

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            Pose position = aprilTagWebcam.relocalize("BLUE AUDIENCE", TagPoses.get(RobotConstants.currentTag));

            if (gamepad1.b && position != null) {
                follower.setPose(position);
            }

            if (position != null) {
                multipleTelemetry.addData("reloc heading deg", Math.toDegrees(position.heading()));
            }
            multipleTelemetry.addData("follower heading deg", Math.toDegrees(follower.pose().heading()));

            if (position != null) {
                multipleTelemetry.addData("reloc heading deg", Math.toDegrees(position.heading()));
            }
            multipleTelemetry.addData("follower heading deg", Math.toDegrees(follower.pose().heading()));
            multipleTelemetry.addData("actual positoin: ", follower.pose());

            telemetry.update();
            multipleTelemetry.update();
            follower.update();

        }

        if (robot.visionPortal != null) {
            robot.visionPortal.close();
        }
    }
}
