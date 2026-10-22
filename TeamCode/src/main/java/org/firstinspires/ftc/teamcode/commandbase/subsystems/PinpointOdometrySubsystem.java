package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import androidx.annotation.NonNull;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

import java.util.Locale;

/**
 * Converts pinpoint-centric odometry data (where the pinpoint defines 0,0,0 at its last reset)
 * into field-centric position and velocity.
 * Make sure that the start position is where the robot was when pinpoint last reset.
 *
 * <p>This subsystem uses the same coordinate and rotation conventions as
 * {@link MecanumDriveSubsystem}, so it works smoothly with PID movement
 * and drive code.</p>
 *
 * <p><b>Important:</b> {@link #update()} must be called for this class to work.
 * Each call makes an I2C read, so you should only call it once in your robot code.</p>
 *
 * <p><b>Note on pinpoint odometry:</b><br>
 * Pose angles are clockwise-positive, while GoBilda encoders are clockwise-negative.
 * Configure odometry so that:
 * <ul>
 *   <li>X increases when moving forward</li>
 *   <li>Y increases when moving left</li>
 * </ul>
 * </p>
 *
 * @see GoBildaPinpointDriver
 * @see MecanumDriveSubsystem
 */

public class PinpointOdometrySubsystem implements Subsystem{
    private final GoBildaPinpointDriver driver;
    private Pose2d startPose;

    /*
     * Explanation of X_OFFSET and Y_OFFSET:
     * Tracking point is the back-left corner of robot
     * These conventions are defined in the GoBilda Pinpoint User Guide.
     */
    private final double X_OFFSET = 1.642850590503122;
    private final double Y_OFFSET = -6.14031513844888;


    /**
     * @param startPose The starting pose of the robot (in field centric coordinates)
     * @param robotHardware Collection of the Robot's Hardware
     */
    public PinpointOdometrySubsystem(RobotHardware robotHardware, Pose2d startPose) {
        this.driver = robotHardware.odo;
        this.startPose = startPose;
    }


    /**
     * Gets the current velocity of the robot in the field centric coordinate system
     * Converts driver velocity (pinpoint relative frame information) to field centric velocity
     */
    public Pose2d getVelocity() {
        Pose2d pinpointCentricVelocity = new Pose2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH), new Rotation2d(driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));
        Pose2d startVelocity = new Pose2d(0, 0, startPose.getRotation());
        return startVelocity.plus(new Transform2d(pinpointCentricVelocity.getTranslation(), pinpointCentricVelocity.getRotation()));
    }

    /**
     * Gets the current position of the robot in the field centric coordinate system
     * Converts driver position (pinpoint relative frame information) to field centric position
     * <p>
     *   See documentation for <a href="https://docs.ftclib.org/ftclib/features/geometry#transform-and-twist">Transform2d</a>
     * </p>
     **/
    public Pose2d getPose() {
        Pose2d pinpointCentricPose = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), new Rotation2d(driver.getHeading(AngleUnit.RADIANS)));
        return startPose.plus(new Transform2d(pinpointCentricPose.getTranslation(), pinpointCentricPose.getRotation()));
    }


    /**
     * Updates the driver to retrieve the latest position data.
     * Call this method repeatedly (e.g., in your OpMode loop) to keep position information current.
     * update() causes an expensive I2C call to be made, so ideally,
     * you should call it once in your entire robot code.
     */
    @Override
    public void update() {
        driver.update();
    }
    @Override
    public void init(){
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        driver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        driver.setOffsets(X_OFFSET, Y_OFFSET, DistanceUnit.INCH);
        driver.resetPosAndIMU();
        driver.update();
    }
    public void resetTo(Pose2d newPose){
        driver.resetPosAndIMU();
        startPose = newPose;
        driver.update();
    }
    public double getDistanceTo(Translation2d other){
        Pose2d robotPose = getPose();
        //this is because pedro-constants reverse y (left) while field is regular y
        Translation2d robot = new Translation2d(robotPose.getX(), -robotPose.getY());
        return robot.getDistance(other);
    }

    /**
     * @return String with x, y (in inches) and heading (RADS) from the subsystem, rounded to 1 decimal.
     */
    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.1f, %.1f, %.1f", getPose().getX(), getPose().getY(), getPose().getHeading());
    }
}


