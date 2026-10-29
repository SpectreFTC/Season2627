package org.firstinspires.ftc.teamcode.commandbase.commands;



import static org.firstinspires.ftc.teamcode.globals.RobotUtils.isInsideCircle;
import static org.firstinspires.ftc.teamcode.globals.RobotUtils.isWithinRange;
import static org.firstinspires.ftc.teamcode.globals.RobotUtils.normalizeAngleInRadians;

//import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.commandbase.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.PinpointOdometrySubsystem;

import java.util.Locale;

/**
 * Command that moves the drivetrain to a target position using PID, based on odometry.
 *
 * <li><p><b>Position Tolerance:</b> Defined as a circular region with a user-specified radius (in inches).</p></li>
 * <li><p><b>Heading tolerance:</b> An absolute threshold in degrees.</p></li>
 *
 * <li><p><b>Waypoints:</b> Intermediate targets used to navigate around obstacles without requiring a full stop,
 * but the probability that the speed of the robot is high when passing through a waypoint decreases as tolerances decreases
 * <li><p><b>Stopping points:</b> Require the robot to be within a radius and heading tolerance,
 * and fully stops the drivetrain by applying brakes. Stopping points also include a target velocity,
 * which forces the drivetrain to slow down (preventing the PID movement from overshooting).
 * This way, when the brakes are applied, the robot has little to no momentum,
 * minimizing drift after the movement ends.</p></li>
 */
//@Config
public class PIDMovement extends CommandBase {
    public static double xkP = 0.4, xkI = 0, xkD = 0.035, xkF = 0.03;
    public static double ykP = 0.45, ykI = 0.0, ykD = 0.05, ykF = 0.01;
    public static double hkP = 4, hkI = 0, hkD = 0.2, hkF = 0.025;
    private final double velocityX = 1;
    private final double velocityY = 1;
    private final double turnVelocity = 1;

    private final PIDFController xPID = new PIDFController(xkP, xkI, xkD, 0);
    private final PIDFController yPID = new PIDFController(ykP, ykI, ykD, 0);
    private final PIDFController hPID = new PIDFController(hkP, hkI, hkD, 0);
    /**
     * Velocity threshold that the drivetrain must be between to stop for stopping points.
     */
    private final Pose2d POSE_VELOCITY_THRESHOLD = new Pose2d(velocityX, velocityY, Rotation2d.fromDegrees(turnVelocity));
    private final double radiusTolerance;
    private final double headingToleranceRadians;
    private Pose2d target;
    private final MecanumDriveSubsystem driveSubsystem;
    private final PinpointOdometrySubsystem odometrySubsystem;
    private final boolean isWaypoint;
    private double headingError;
    private double yError;
    private double xError;
    private boolean isWithinThreshold;
    private boolean isInsideCircle;
    private double xPower;
    private double yPower;
    private double hPower;
    private double maxPower;
    /**
     * <font size="4"><b>The parameters for the constructor are:</b></font><br>
     *
     * @param odometrySubsystem Odometry information.
     * @param driveSubsystem    The drive subsystem, providing access to methods like driveFieldCentric.
     * @param x                 The x value in inches to reach.
     * @param y                 The y value in inches to reach.
     * @param h                 The heading in degrees to reach.
     * @param headingThreshold  The heading tolerance in degrees.
     * @param radiusTolerance   The radius (inches) defines a unit circle where the target is the center.
     *                          This command ends when the robot has reached the tolerance that was specified.
     *                          You can think of this as a rough target the robot has to reach.
     * @param isWaypoint        Specifies whether the movement is a waypoint
     *                          (Intermediate targets used to navigate around obstacles without requiring a full stop)
     * @param maxPower          The maximum power the drivetrain runs at, this applies to all wheels equally
     */
    public PIDMovement(double x,
                       double y,
                       double h,
                       MecanumDriveSubsystem driveSubsystem,
                       PinpointOdometrySubsystem odometrySubsystem,
                       double headingThreshold,
                       double radiusTolerance,
                       boolean isWaypoint,
                       double maxPower) {
        this.isWaypoint = isWaypoint;
        this.headingToleranceRadians = Math.toRadians(headingThreshold);
        this.radiusTolerance = radiusTolerance;
        this.driveSubsystem = driveSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.target = new Pose2d(x, y, Rotation2d.fromDegrees(h));
        this.maxPower = maxPower;
    }
    /**
     * <font size="4"><b>The parameters for the constructor are:</b></font><br>
     *
     * @param odometrySubsystem Odometry information.
     * @param driveSubsystem    The drive subsystem, providing access to methods like driveFieldCentric.
     * @param x                 The x value in inches to reach.
     * @param y                 The y value in inches to reach.
     * @param h                 The heading in degrees to reach.
     * @param headingThreshold  The heading tolerance in degrees.
     * @param radiusTolerance   The radius (inches) defines a unit circle where the target is the center.
     *                          This command ends when the robot has reached the tolerance that was specified.
     *                          You can think of this as a rough target the robot has to reach.
     * @param isWaypoint        Specifies whether the movement is a waypoint
     *                          (Intermediate targets used to navigate around obstacles without requiring a full stop)
     */
    public PIDMovement(double x,
                       double y,
                       double h,
                       MecanumDriveSubsystem driveSubsystem,
                       PinpointOdometrySubsystem odometrySubsystem,
                       double headingThreshold,
                       double radiusTolerance,
                       boolean isWaypoint) {
        this(x,y,h, driveSubsystem, odometrySubsystem, headingThreshold, radiusTolerance, isWaypoint, 1);
    }

    @Override
    public void execute() {
        Pose2d currentPosition = odometrySubsystem.getPose();
        double deltaHeadingRadians = normalizeAngleInRadians(currentPosition.getHeading() - target.getHeading());
        double xPower = 0;
        double yPower = 0;
        double hPower = 0;

        // apply PID powers in x,y,h
        xPower += xPID.calculate(currentPosition.getX(), target.getX());
        yPower += yPID.calculate(currentPosition.getY(), target.getY());
        hPower += hPID.calculate(deltaHeadingRadians, 0);

        // apply kF in x,y,h
        // kF is a constant power applied to the drivetrain in the direction of motion to help overcome friction.
        // Think of this value as the minimum power required to get the drivetrain moving.
        xPower += xkF * Math.signum(target.getX()- currentPosition.getX());
        yPower += ykF * Math.signum(target.getY()- currentPosition.getY());
        hPower += hkF * Math.signum(-deltaHeadingRadians);

        // limit maximum power applied
        xPower = Range.clip(xPower, -maxPower, maxPower);
        yPower = Range.clip(yPower, -maxPower, maxPower);
        hPower = Range.clip(hPower, -maxPower, maxPower);

        driveSubsystem.driveFieldCentric(xPower, yPower, hPower, currentPosition.getHeading());
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPosition = odometrySubsystem.getPose();
        Pose2d currentVelocity = odometrySubsystem.getVelocity();
        xError = (target.getX() - currentPosition.getX());
        yError = (target.getY() - currentPosition.getY());
        isInsideCircle = isInsideCircle(xError, yError, radiusTolerance);
        headingError = normalizeAngleInRadians(
                target.getHeading() - currentPosition.getHeading()
        );
        isWithinThreshold = isWithinRange(headingError, -headingToleranceRadians, headingToleranceRadians);
        if (!isInsideCircle || !isWithinThreshold) {
            return false;
        }

        if (isWaypoint) {
            return true;
        }

        if (currentVelocity.getX() <= POSE_VELOCITY_THRESHOLD.getX() &&
                currentVelocity.getY() <= POSE_VELOCITY_THRESHOLD.getY() &&
                currentVelocity.getHeading() <= POSE_VELOCITY_THRESHOLD.getHeading()) {
            driveSubsystem.turnOffMotors();
            return true;
        }

        return false;

    }

    public String toString() {
        return String.format(Locale.ENGLISH, "%.1f, %.1f, %.1f, %b, %b, %.1f, %.1f, %.1f", xPower, yPower, hPower, isInsideCircle, isWithinThreshold, xError, yError, headingError);
    }

}
