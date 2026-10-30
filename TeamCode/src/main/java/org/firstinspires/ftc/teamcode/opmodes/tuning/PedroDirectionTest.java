package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Checks that Pedro's start pose, drive motors and odometry all agree, using the exact
 * Constants the autos use. Put the robot in open space, facing any direction.
 *
 * <ol>
 *   <li>Before START: heading should read 90. If it reads 0 (or anything else), the start
 *       pose isn't reaching the Pinpoint.</li>
 *   <li>Hold A: the robot should drive FORWARD, Y should go up and X stay the same.</li>
 *   <li>Hold B: the robot should strafe LEFT, X should go down.</li>
 *   <li>Hold X: the robot should turn LEFT (counterclockwise from above), heading should go up.</li>
 * </ol>
 * Nothing pressed: the robot stops. Pedro only drives the motors when a button is held.
 */
@TeleOp(group = "TEST", name = "Pedro Direction Test")
public class PedroDirectionTest extends OpMode {
    private static final double POWER = 0.3;

    Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Same kind of start pose as the autos: facing +y
        follower.setPose(PoseFactory.degrees().of(72, 72, 90));
    }

    @Override
    public void init_loop() {
        // Safe in INIT: the follower is idle, so update() only reads odometry and stops the motors
        follower.update();
        telemetry.addLine("Before START, heading should read 90.");
        showPose();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            follower.manual(POWER, 0, 0);   // forward
        } else if (gamepad1.b) {
            follower.manual(0, POWER, 0);   // strafe left
        } else if (gamepad1.x) {
            follower.manual(0, 0, POWER);   // turn left
        } else {
            follower.manual(0, 0, 0);
        }
        follower.update();

        telemetry.addLine("Hold A: forward, Y up. B: strafe left, X down. X: turn left, heading up.");
        showPose();
    }

    @Override
    public void stop() {
        follower.stop();
    }

    private void showPose() {
        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }
}