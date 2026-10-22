package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.globals.BulkCache;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

/**
 * Manual shot-table tuner: finds how fast the flywheel should spin and where the hood
 * should sit for each distance, then fits the two lines ShooterSubsystem uses:
 * <pre>
 *     velocity  = mVelocity  * distance + bVelocity
 *     hoodAngle = mHoodAngle * distance + bHoodAngle
 * </pre>
 * <p>The shooter runs through {@code ShooterSubsystem.runAtVelocity()} and
 * {@code setHoodAngle()}, the same code used in matches.</p>
 *
 * <h2>How to use it</h2>
 * <ol>
 *   <li>Put the robot at a known distance from the goal and set {@code DISTANCE_INCHES}
 *       in FTC Dashboard. Measure it the same way the match code's distance supplier
 *       does (same points on the robot and the goal), or the fit will be off by that
 *       difference.</li>
 *   <li>Adjust velocity (dpad up/down) and hood angle (dpad left/right) and shoot until
 *       shots go in reliably. Shoot only when telemetry says READY.</li>
 *   <li>Press X to record the point. Move to a new distance and repeat. Cover the whole
 *       range you'll shoot from in matches, with at least 3 distances.</li>
 *   <li>Once there are 2 or more distances, the fit is applied to RobotConstants for this
 *       app session. Press Y to switch to FIT mode and test it at distances you didn't
 *       record.</li>
 *   <li>Copy the four numbers into RobotConstants.java to keep them.</li>
 * </ol>
 *
 * <h2>Controls (gamepad 1)</h2>
 * <ul>
 *   <li>Sticks: drive, at DRIVE_SPEED for fine aiming.</li>
 *   <li>Left trigger, left bumper, right bumper: intake, feed and flower, same as Teleop.</li>
 *   <li>Dpad up/down: velocity +/- VELOCITY_STEP. Dpad right/left: hood +/- HOOD_STEP.
 *       Setting VELOCITY or HOOD_ANGLE in Dashboard also works.</li>
 *   <li>A: flywheel on/off. X: record a point. B: delete the last point.
 *       Y: switch between MANUAL and FIT.</li>
 * </ul>
 * <p>Recorded points are kept until the Robot Controller app restarts, so stopping and
 * restarting this OpMode doesn't lose them. Set {@code CLEAR_POINTS} to true in
 * Dashboard to start over. Each point is also written to the robot log.</p>
 */
@Config
@TeleOp
public class ShooterDistanceTuner extends OpMode {

    // ---- Settings (editable from FTC Dashboard) ----
    public static double DISTANCE_INCHES = 36;   // measured the same way as the match code's distance
    public static double VELOCITY = 1000;        // ticks/sec; a new value here replaces the dpad value
    public static double HOOD_ANGLE = RobotConstants.hoodServoAngleAtCalibration;  // same units as setHoodAngle()
    public static double VELOCITY_STEP = 20;     // ticks/sec per dpad up/down press
    public static double HOOD_STEP = 1;          // per dpad left/right press
    public static double READY_TOLERANCE_TPS = 30;  // READY when the speed is this close to the target
    public static double DRIVE_SPEED = 0.5;      // stick scale, so small aiming moves are easy
    public static boolean CLEAR_POINTS = false;  // set true to delete every recorded point

    // The model in RobotConstants.java when the app started, restored if the fit is cleared
    private static final double FILE_M_VELOCITY = RobotConstants.mVelocity;
    private static final double FILE_B_VELOCITY = RobotConstants.bVelocity;
    private static final double FILE_M_HOOD = RobotConstants.mHoodAngle;
    private static final double FILE_B_HOOD = RobotConstants.bHoodAngle;

    // Kept across OpMode runs until the app restarts
    private static final List<ShotPoint> points = new ArrayList<>();

    private enum Mode { MANUAL, FIT }

    RobotHardware robotHardware;
    ShooterSubsystem shooter;
    MecanumDriveSubsystem drive;
    MultipleTelemetry multipleTelemetry;
    BulkCache bulkCache;
    ElapsedTime loopTimer;
    Supplier<Double> voltageSupplier;

    private Mode mode = Mode.MANUAL;
    private boolean flywheelOn = true;
    private boolean flowerDown = false;
    private String message = "";

    // Manual values, changed by the dpad or by Dashboard
    private double manualVelocity, manualHood;
    private double lastDashboardVelocity, lastDashboardHood;

    // What the shooter is being told this loop
    private double targetVelocity, targetHood;

    // ---- Fit ----
    private boolean fitValid = false;
    private double velocitySlope, velocityIntercept, velocityR2;
    private double hoodSlope, hoodIntercept, hoodR2;

    // ---- Button edge detection ----
    private static final int UP = 0, DOWN = 1, RIGHT = 2, LEFT = 3, A = 4, B = 5, X = 6, Y = 7, RB = 8;
    private final boolean[] lastButtons = new boolean[9];

    @Override
    public void init() {
        // Clear subsystems registered by the last OpMode run
        CommandScheduler.getInstance().reset();

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware = RobotHardware.getInstance(this);
        shooter = new ShooterSubsystem(robotHardware.hoodServo, robotHardware.shooterMotor1, robotHardware.shooterMotor2);
        drive = new MecanumDriveSubsystem(robotHardware);

        // After RobotHardware, so its bulk caching mode isn't overwritten
        bulkCache = new BulkCache(hardwareMap);

        // ShooterSubsystem only calls this every 50 ms; it isn't in the bulk data
        voltageSupplier = robotHardware.voltageSensor::getVoltage;

        manualVelocity = lastDashboardVelocity = VELOCITY;
        manualHood = lastDashboardHood = HOOD_ANGLE;

        loopTimer = new ElapsedTime();

        // Points from an earlier run of this OpMode are still here
        fit();

        multipleTelemetry.addLine("Shooter distance tuner. The flywheel spins up on START (A toggles it).");
        multipleTelemetry.addData("Points kept from earlier runs", points.size());
        multipleTelemetry.addLine("Keep hands clear.");
        multipleTelemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void loop() {
        // Fresh bulk data every loop; without this, the cached velocity would never update
        bulkCache.clearCache();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // Read every button once per loop, so edge detection sees every change
        boolean up = pressed(UP, gamepad1.dpad_up);
        boolean down = pressed(DOWN, gamepad1.dpad_down);
        boolean right = pressed(RIGHT, gamepad1.dpad_right);
        boolean left = pressed(LEFT, gamepad1.dpad_left);
        boolean a = pressed(A, gamepad1.a);
        boolean b = pressed(B, gamepad1.b);
        boolean x = pressed(X, gamepad1.x);
        boolean y = pressed(Y, gamepad1.y);
        boolean rightBumper = pressed(RB, gamepad1.right_bumper);

        // ---- Drive, slowed down for aiming ----
        drive.driveRobotCentric(
                -gamepad1.left_stick_y * DRIVE_SPEED,
                gamepad1.left_stick_x * DRIVE_SPEED,
                gamepad1.right_stick_x * DRIVE_SPEED);

        // ---- Intake, feed and flower, same as Teleop ----
        if (gamepad1.left_trigger > 0) {
            robotHardware.intakeMotor.setPower(1);
            robotHardware.transferMotor.setPower(0);
        } else if (gamepad1.left_bumper) {
            robotHardware.intakeMotor.setPower(-1);
            robotHardware.transferMotor.setPower(1);
        } else {
            robotHardware.intakeMotor.setPower(0);
            robotHardware.transferMotor.setPower(0);
        }
        if (rightBumper) flowerDown = !flowerDown;
        robotHardware.flowerServo.setPosition(flowerDown ? 0.5 : 1);

        // ---- Manual values: a Dashboard edit replaces them, the dpad steps them ----
        if (VELOCITY != lastDashboardVelocity) {
            manualVelocity = VELOCITY;
            lastDashboardVelocity = VELOCITY;
        }
        if (HOOD_ANGLE != lastDashboardHood) {
            manualHood = HOOD_ANGLE;
            lastDashboardHood = HOOD_ANGLE;
        }
        if (up) manualVelocity += VELOCITY_STEP;
        if (down) manualVelocity = Math.max(0, manualVelocity - VELOCITY_STEP);
        if (right) manualHood += HOOD_STEP;
        if (left) manualHood -= HOOD_STEP;

        // ---- Buttons ----
        if (a) flywheelOn = !flywheelOn;
        if (y) {
            mode = mode == Mode.MANUAL ? Mode.FIT : Mode.MANUAL;
            message = mode == Mode.FIT
                    ? (fitValid ? "FIT: using the fitted lines." : "FIT: using RobotConstants.java (no fit yet).")
                    : "MANUAL: using your velocity and hood.";
        }
        if (x) recordPoint();
        if (b) deleteLastPoint();
        if (CLEAR_POINTS) {
            CLEAR_POINTS = false;
            points.clear();
            fit();
            message = "Cleared all points.";
        }

        // ---- Shooter ----
        if (mode == Mode.FIT) {
            // Same formula as ShooterSubsystem.shooterSetVelocityRunCommand()
            targetVelocity = RobotConstants.mVelocity * DISTANCE_INCHES + RobotConstants.bVelocity;
            targetHood = RobotConstants.mHoodAngle * DISTANCE_INCHES + RobotConstants.bHoodAngle;
        } else {
            targetVelocity = manualVelocity;
            targetHood = manualHood;
        }
        shooter.setHoodAngle(targetHood);
        shooter.runAtVelocity(flywheelOn ? targetVelocity : 0, voltageSupplier);

        sendTelemetry(loopMs);
    }

    @Override
    public void stop() {
        shooter.stop();
        robotHardware.intakeMotor.setPower(0);
        robotHardware.transferMotor.setPower(0);
        drive.driveRobotCentric(0, 0, 0);
    }

    // ------------------------------------------------------------------
    // Points and fit
    // ------------------------------------------------------------------

    private void recordPoint() {
        if (mode == Mode.FIT) {
            message = "Switch to MANUAL (Y) to record. FIT mode is only for testing the fit.";
            return;
        }
        boolean duplicate = false;
        for (ShotPoint p : points) {
            if (Math.abs(p.distance - DISTANCE_INCHES) < 0.5) duplicate = true;
        }

        points.add(new ShotPoint(DISTANCE_INCHES, manualVelocity, manualHood));
        RobotLog.ii("ShooterDistanceTune", "point %d: distance %.1f in, velocity %.0f tps, hood %.2f",
                points.size(), DISTANCE_INCHES, manualVelocity, manualHood);
        fit();

        message = String.format(Locale.US, "Recorded point %d at %.1f in.", points.size(), DISTANCE_INCHES);
        if (duplicate) {
            message += " There was already a point at this distance; both are kept."
                    + " Did you update DISTANCE_INCHES?";
        }
    }

    private void deleteLastPoint() {
        if (points.isEmpty()) {
            message = "No points to delete.";
            return;
        }
        ShotPoint removed = points.remove(points.size() - 1);
        fit();
        message = String.format(Locale.US, "Deleted the point at %.1f in.", removed.distance);
    }

    /**
     * Least-squares lines through the points. With a valid fit (2+ different distances),
     * it is applied to RobotConstants for this app session, so FIT mode and the match
     * code use it right away. Otherwise RobotConstants goes back to the file's values.
     */
    private void fit() {
        SimpleRegression velocityFit = new SimpleRegression();
        SimpleRegression hoodFit = new SimpleRegression();
        for (ShotPoint p : points) {
            velocityFit.addData(p.distance, p.velocity);
            hoodFit.addData(p.distance, p.hood);
        }

        velocitySlope = velocityFit.getSlope();
        velocityIntercept = velocityFit.getIntercept();
        velocityR2 = velocityFit.getRSquare();
        hoodSlope = hoodFit.getSlope();
        hoodIntercept = hoodFit.getIntercept();
        hoodR2 = hoodFit.getRSquare();

        // The slope is NaN with fewer than 2 points or when every point has the same distance
        fitValid = points.size() >= 2 && isFinite(velocitySlope) && isFinite(hoodSlope);

        if (fitValid) {
            RobotConstants.mVelocity = velocitySlope;
            RobotConstants.bVelocity = velocityIntercept;
            RobotConstants.mHoodAngle = hoodSlope;
            RobotConstants.bHoodAngle = hoodIntercept;
        } else {
            RobotConstants.mVelocity = FILE_M_VELOCITY;
            RobotConstants.bVelocity = FILE_B_VELOCITY;
            RobotConstants.mHoodAngle = FILE_M_HOOD;
            RobotConstants.bHoodAngle = FILE_B_HOOD;
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /** True only on the loop the button goes down. */
    private boolean pressed(int index, boolean now) {
        boolean was = lastButtons[index];
        lastButtons[index] = now;
        return now && !was;
    }

    // ------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------

    private void sendTelemetry(double loopMs) {
        // Read here, not from the subsystem, so it still updates while the flywheel coasts.
        // Same sign as ShooterSubsystem; free, since it comes from the bulk cache.
        double velocity = -robotHardware.shooterMotor1.getVelocity();
        boolean ready = flywheelOn && targetVelocity > 0
                && Math.abs(velocity - targetVelocity) <= READY_TOLERANCE_TPS;

        multipleTelemetry.addData("Mode", mode == Mode.MANUAL ? "MANUAL (Y for FIT)" : "FIT (Y for MANUAL)");
        multipleTelemetry.addData("Flywheel", !flywheelOn ? "OFF (A to start)"
                : ready ? "READY" : "spinning up, wait");
        multipleTelemetry.addData("Distance (in)", DISTANCE_INCHES);

        // Plain numbers so they can be graphed in FTC Dashboard
        multipleTelemetry.addData("Target (tps)", targetVelocity);
        multipleTelemetry.addData("Velocity (tps)", velocity);
        multipleTelemetry.addData("Hood angle", targetHood);
        multipleTelemetry.addData("Loop (ms)", loopMs);

        if (!message.isEmpty()) {
            multipleTelemetry.addLine();
            multipleTelemetry.addLine(message);
        }

        multipleTelemetry.addLine();
        multipleTelemetry.addLine(String.format(Locale.US, "Points (%d):", points.size()));
        for (int i = 0; i < points.size(); i++) {
            ShotPoint p = points.get(i);
            String line = String.format(Locale.US, "  %d: %.1f in, %.0f tps, hood %.2f",
                    i + 1, p.distance, p.velocity, p.hood);
            if (fitValid) {
                // How far the line misses this point; a big one means a bad point or a curve
                line += String.format(Locale.US, "  (fit off by %+.0f tps, %+.2f)",
                        velocitySlope * p.distance + velocityIntercept - p.velocity,
                        hoodSlope * p.distance + hoodIntercept - p.hood);
            }
            multipleTelemetry.addLine(line);
        }

        multipleTelemetry.addLine();
        if (fitValid) {
            multipleTelemetry.addLine(String.format(Locale.US, "Fit R-squared: velocity %s, hood %s",
                    formatR2(velocityR2), formatR2(hoodR2)));
            multipleTelemetry.addLine("Applied for this session. Copy into RobotConstants.java:");
            multipleTelemetry.addLine(String.format(Locale.US,
                    "  mVelocity = %.6g;  bVelocity = %.6g;", velocitySlope, velocityIntercept));
            multipleTelemetry.addLine(String.format(Locale.US,
                    "  mHoodAngle = %.6g;  bHoodAngle = %.6g;", hoodSlope, hoodIntercept));
        } else {
            multipleTelemetry.addLine("Record at least 2 different distances to fit.");
        }

        multipleTelemetry.update();
    }

    private static String formatR2(double r2) {
        // NaN when every point has the same value (a flat line fits perfectly)
        return Double.isNaN(r2) ? "-" : String.format(Locale.US, "%.3f", r2);
    }

    // ------------------------------------------------------------------
    // Data
    // ------------------------------------------------------------------

    static class ShotPoint {
        final double distance;  // inches
        final double velocity;  // ticks/sec
        final double hood;      // same units as setHoodAngle()

        ShotPoint(double distance, double velocity, double hood) {
            this.distance = distance;
            this.velocity = velocity;
            this.hood = hood;
        }
    }
}