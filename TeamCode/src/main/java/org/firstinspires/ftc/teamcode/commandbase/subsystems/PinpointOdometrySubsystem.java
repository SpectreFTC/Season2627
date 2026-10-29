package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

import java.util.Locale;

/**
 * Field-centric robot pose from the goBILDA Pinpoint.
 *
 * <h2>How it works</h2>
 * <p>The Pinpoint tracks the robot relative to wherever it was last reset: x forward, y left,
 * heading counterclockwise-positive. {@code startPose} is the robot's field pose at that
 * moment, and {@link #getPose()} adds the Pinpoint's movement onto it (rotated by the start
 * heading) to get the field pose.</p>
 *
 * <h2>Field frame (BIOBUZZ)</h2>
 * <p>Origin at field center. +x points away from the audience, +y points toward the blue
 * goals. Heading 0 faces +x, turning left is positive, and it wraps from +180 to -180
 * degrees.</p>
 * <p>Because heading is counterclockwise-positive, +y has to be 90 degrees to the left of +x.
 * Standing on the audience side looking at the field, the blue goals must be on your left. If
 * they're on your right, point +y at the red goals instead, or every pose will turn the wrong
 * way once the robot rotates.</p>
 *
 * <h2>Checking the setup</h2>
 * <p>Start an OpMode with a start pose of {@code new Pose2d()} and push the robot by hand.
 * Forward should raise x, left should raise y, and turning left should raise the heading.
 * If y goes the wrong way, flip {@link #Y_POD_DIRECTION}; if x does, flip
 * {@link #X_POD_DIRECTION}. Don't fix it by negating values in code: the Pinpoint combines
 * both pods with the heading internally, so a backwards pod also corrupts x once the robot
 * turns.</p>
 *
 * <p>Call {@link #update()} once per loop, before anything reads the pose. It's the only
 * regular I2C read; {@link #resetTo} adds one more, only when relocalizing.</p>
 */
public class PinpointOdometrySubsystem implements Subsystem {

    // Pod positions relative to the robot's center (the point this class tracks), in inches.
    // X pod offset: how far LEFT of center the forward pod is (right of center is negative).
    // Y pod offset: how far FORWARD of center the strafe pod is (behind center is negative).
    // These should match the pod offsets in your Pedro constants.
    private static final double X_POD_OFFSET = 1.642850590503122;
    private static final double Y_POD_OFFSET = -6.14031513844888;

    // The forward pod must count up moving forward; the strafe pod must count up moving left.
    // Pedro and this class configure the same Pinpoint, so these must match your Pedro
    // constants, or teleop undoes Pedro's settings every time it starts.
    // Y is REVERSED because the old code had to flip y to get sensible distances.
    // Confirm with the push test above.
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    private final GoBildaPinpointDriver driver;

    // The robot's field pose at the moment the Pinpoint's own pose was (0, 0, 0)
    private Pose2d startPose;

    /**
     * @param robotHardware collection of the robot's hardware
     * @param startPose     the robot's field pose when {@link #init()} runs
     */
    public PinpointOdometrySubsystem(RobotHardware robotHardware, Pose2d startPose) {
        this.driver = robotHardware.odo;
        this.startPose = startPose;
    }

    @Override
    public void init() {
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        driver.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        driver.setOffsets(X_POD_OFFSET, Y_POD_OFFSET, DistanceUnit.INCH);
        // Zeroes the position and recalibrates the IMU. That takes about 0.25 s and the robot
        // must be still, which is fine during INIT but not mid-match (see resetTo).
        driver.resetPosAndIMU();
        driver.update();
    }

    /** Reads the Pinpoint. Call once per loop, before anything reads the pose. */
    @Override
    public void update() {
        driver.update();
    }

    /** The robot's field pose as of the last {@link #update()}. */
    public Pose2d getPose() {
        return startPose.transformBy(sinceStart());
    }

    /**
     * Tells the subsystem the robot is at {@code newPose} right now, e.g. when relocalizing
     * in a corner. Recalculates {@code startPose} so the current reading comes out as
     * newPose. The Pinpoint itself isn't reset, so its IMU isn't recalibrated: there's no
     * 0.25 s wait and no risk of a bad gyro zero if the robot is still moving.
     */
    public void resetTo(Pose2d newPose) {
        driver.update();  // freshest reading, so newPose lines up with where the robot is now
        startPose = newPose.transformBy(sinceStart().inverse());
    }

    /** Straight-line distance from the robot's center to a field point, in inches. */
    public double getDistanceTo(Translation2d point) {
        return getPose().getTranslation().getDistance(point);
    }

    /**
     * Field-centric velocity in inches/sec, rotated the same way as the position. This
     * assumes the Pinpoint reports velocity in the same frame as its position; goBILDA's
     * docs don't say. The heading part is the turn rate, but a Rotation2d wraps it past
     * 180 degrees/sec, so use {@link #getHeadingVelocity()} for the real value.
     */
    public Pose2d getVelocity() {
        Translation2d fieldVelocity = new Translation2d(
                driver.getVelX(DistanceUnit.INCH),
                driver.getVelY(DistanceUnit.INCH)
        ).rotateBy(startPose.getRotation());
        return new Pose2d(fieldVelocity, new Rotation2d(getHeadingVelocity()));
    }

    /** Turn rate in radians/sec, counterclockwise-positive, not wrapped. */
    public double getHeadingVelocity() {
        return driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    /** The Pinpoint's own pose since its last reset, as a move from startPose. */
    private Transform2d sinceStart() {
        return new Transform2d(
                new Translation2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH)),
                new Rotation2d(driver.getHeading(AngleUnit.RADIANS))
        );
    }

    /**
     * @return x, y (inches) and heading (radians) of the field pose, rounded to 1 decimal
     */
    @NonNull
    @Override
    public String toString() {
        Pose2d pose = getPose();
        return String.format(Locale.ENGLISH, "%.1f, %.1f, %.1f", pose.getX(), pose.getY(), pose.getHeading());
    }
}