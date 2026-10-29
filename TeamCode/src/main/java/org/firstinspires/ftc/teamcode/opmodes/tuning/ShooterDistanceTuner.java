package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.PinpointOdometrySubsystem;
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
 * <p>The distance comes from Pinpoint odometry, calculated the same way as MainTeleop:
 * {@code getDistanceTo(goal) - 22}. The robot starts at {@code new Pose2d()} (field
 * center), so it has to be relocalized at one of MainTeleop's corner poses before
 * recording. The shooter and intake run through ShooterSubsystem and IntakeSubsystem,
 * the same code and buttons as MainTeleop, so shots are fed the way they are in
 * matches.</p>
 *
 * <h2>How to use it</h2>
 * <ol>
 *   <li>Set {@code BLUE_ALLIANCE} in FTC Dashboard. Put the robot in a corner and press
 *       that corner's button on gamepad 2 (A, B, X or Y) to relocalize. Pick the goal with
 *       gamepad 2 dpad right/left, the same as MainTeleop.</li>
 *   <li>Drive to a shooting spot. Adjust velocity (dpad up/down) and hood angle (dpad
 *       left/right) and shoot (right trigger) until shots go in reliably. Shoot only when
 *       telemetry says READY.</li>
 *   <li>Press X to record the point at the current odometry distance. Move to a new
 *       distance and repeat. Cover the whole range you'll shoot from in matches, with at
 *       least 3 distances.</li>
 *   <li>Once there are 2 or more distances, the fit is applied to RobotConstants for this
 *       app session. Press Y to switch to FIT mode and drive around: the shooter follows
 *       the fitted lines using the live distance, the same as in a match.</li>
 *   <li>Copy the four numbers into RobotConstants.java to keep them.</li>
 * </ol>
 *
 * <h2>Controls</h2>
 * <p>Gamepad 1:</p>
 * <ul>
 *   <li>Sticks: drive robot-centric, at DRIVE_SPEED for fine aiming.</li>
 *   <li>Left trigger: intake. Right trigger: shoot. Left bumper: eject. Right bumper:
 *       flower up/down. Same as MainTeleop.</li>
 *   <li>Dpad up/down: velocity +/- VELOCITY_STEP. Dpad right/left: hood +/- HOOD_STEP.
 *       Setting VELOCITY or HOOD_ANGLE in Dashboard also works.</li>
 *   <li>A: flywheel on/off. X: record a point. B: delete the last point.
 *       Y: switch between MANUAL and FIT.</li>
 * </ul>
 * <p>Gamepad 2:</p>
 * <ul>
 *   <li>A, B, X, Y: relocalize at RA, RS, BA, BS (the same poses as MainTeleop's
 *       gamepad 1 A, B, X, Y).</li>
 *   <li>Dpad right/left: goal on the right (T1) or left (T2), from the driver's view.</li>
 * </ul>
 * <p>Recorded points are kept until the Robot Controller app restarts, so stopping and
 * restarting this OpMode doesn't lose them. Set {@code CLEAR_POINTS} to true in
 * Dashboard to start over. Each point is also written to the robot log, with the pose
 * it was recorded at.</p>
 */
@Disabled
@Config
@TeleOp
public class ShooterDistanceTuner extends OpMode {

    // ---- Settings (editable from FTC Dashboard) ----
    public static boolean BLUE_ALLIANCE = true;  // which pair of goals to shoot at
    public static double VELOCITY = 1000;        // ticks/sec; a new value here replaces the dpad value
    public static double HOOD_ANGLE = RobotConstants.hoodServoAngleAtCalibration;  // same units as setHoodAngle()
    public static double VELOCITY_STEP = 20;     // ticks/sec per dpad up/down press
    public static double HOOD_STEP = 1;          // per dpad left/right press
    public static double READY_TOLERANCE_TPS = 30;  // READY when the speed is this close to the target
    public static double DRIVE_SPEED = 0.5;      // stick scale, so small aiming moves are easy
    public static boolean CLEAR_POINTS = false;  // set true to delete every recorded point

    // Shooter distance = distance to the goal minus this. Must match the "- 22" in
    // MainTeleop's distance supplier, or the fitted lines will be off by mVelocity * 22.
    private static final double DISTANCE_OFFSET_INCHES = 0;

    // Same field positions as MainTeleop (inches, field center = 0, 0)
    Pose2d RESET_RA = new Pose2d(63.182, -69.812, Rotation2d.fromDegrees(-90)),
            RESET_RS = new Pose2d(63.812, 69.812, Rotation2d.fromDegrees(-90)),
            RESET_BS = new Pose2d(-63.182, 69.812, Rotation2d.fromDegrees(90)),
            RESET_BA = new Pose2d(-69.7, -54.7, Rotation2d.fromDegrees(90));
    Translation2d goalPosBS = new Translation2d(-12.758, 14.19), goalPosBA = new Translation2d(-12.758, -14.19);
    Translation2d goalPosRS = new Translation2d(12.742, 14.19), goalPosRA = new Translation2d(12.742,-14.19);

    // A new point this close to an earlier one gets a note, in case X was pressed twice
    private static final double NEAR_INCHES = 2;

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
    IntakeSubsystem intake;
    MecanumDriveSubsystem drive;
    PinpointOdometrySubsystem odo;
    MultipleTelemetry multipleTelemetry;
    BulkCache bulkCache;
    ElapsedTime loopTimer;
    Supplier<Double> voltageSupplier;

    private Mode mode = Mode.MANUAL;
    private boolean flywheelOn = true;
    private boolean flowerDown = false;
    private String message = "";

    // ---- Odometry ----
    // Not static: every run starts at new Pose2d(), so every run has to relocalize
    private boolean localized = false;
    private String resetName = "";
    private boolean rightGoal = true;  // T1, the goal on the right from the driver's view
    private Pose2d pose = new Pose2d();
    private double goalDistance;       // straight from getDistanceTo()
    private double distance;           // goalDistance - DISTANCE_OFFSET_INCHES; what gets recorded

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
    private static final int UP = 0, DOWN = 1, RIGHT = 2, LEFT = 3, A = 4, B = 5, X = 6, Y = 7, RB = 8,
            G2_A = 9, G2_B = 10, G2_X = 11, G2_Y = 12, G2_RIGHT = 13, G2_LEFT = 14;
    private final boolean[] lastButtons = new boolean[15];

    @Override
    public void init() {
        // Clear subsystems registered by the last OpMode run
        CommandScheduler.getInstance().reset();

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware = RobotHardware.getInstance(this);
        shooter = new ShooterSubsystem(robotHardware.hoodServo, robotHardware.shooterMotor1, robotHardware.shooterMotor2);
        intake = new IntakeSubsystem(robotHardware);
        drive = new MecanumDriveSubsystem(robotHardware);
        odo = new PinpointOdometrySubsystem(robotHardware, new Pose2d());

        // Same order as MainTeleop
        shooter.init();
        intake.init();
        drive.init();
        odo.init();

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
        multipleTelemetry.addLine("Relocalize with gamepad 2 A/B/X/Y before recording.");
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
        boolean g2a = pressed(G2_A, gamepad2.a);
        boolean g2b = pressed(G2_B, gamepad2.b);
        boolean g2x = pressed(G2_X, gamepad2.x);
        boolean g2y = pressed(G2_Y, gamepad2.y);
        boolean g2Right = pressed(G2_RIGHT, gamepad2.dpad_right);
        boolean g2Left = pressed(G2_LEFT, gamepad2.dpad_left);

        // ---- Odometry first, so everything below uses this loop's pose ----
        odo.update();
        if (g2a) {
            relocalize(RESET_RA, "RA");
        } else if (g2b) {
            relocalize(RESET_RS, "RS");
        } else if (g2x) {
            relocalize(RESET_BA, "BA");
        } else if (g2y) {
            relocalize(RESET_BS, "BS");
        }
        if (g2Right) rightGoal = true;
        if (g2Left) rightGoal = false;

        // Same as MainTeleop's distance supplier
        goalDistance = odo.getDistanceTo(currentGoal());
        distance = goalDistance - DISTANCE_OFFSET_INCHES;
        pose = odo.getPose();

        // ---- Drive, slowed down for aiming ----
        drive.driveRobotCentric(
                -gamepad1.left_stick_y * DRIVE_SPEED,
                gamepad1.left_stick_x * DRIVE_SPEED,
                gamepad1.right_stick_x * DRIVE_SPEED);
        drive.update();

        // ---- Intake, same buttons and methods as MainTeleop ----
        if (gamepad1.left_trigger > 0) {
            intake.intake();
        } else if (gamepad1.left_bumper) {
            intake.eject();
        } else if (gamepad1.right_trigger > 0) {
            intake.shoot();
        } else {
            intake.hold();
        }
        if (rightBumper) flowerDown = !flowerDown;
        if (flowerDown) {
            intake.flowerIntakeRampDown();
        } else {
            intake.flowerIntakeRampUp();
        }
        intake.update();

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
            // Same formula as ShooterSubsystem.shooterSetVelocityRunCommand(), with the live distance
            targetVelocity = RobotConstants.mVelocity * distance + RobotConstants.bVelocity;
            targetHood = RobotConstants.mHoodAngle * distance + RobotConstants.bHoodAngle;
        } else {
            targetVelocity = manualVelocity;
            targetHood = manualHood;
        }
        shooter.setHoodAngle(targetHood);
        shooter.runAtVelocity(flywheelOn ? targetVelocity : 0, voltageSupplier);
        shooter.update();

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
    // Odometry
    // ------------------------------------------------------------------

    private void relocalize(Pose2d resetPose, String name) {
        odo.resetTo(resetPose);
        localized = true;
        resetName = name;
        message = "Relocalized at " + name + ".";
    }

    /** Same goals as MainTeleop: T1 on the right, T2 on the left, from the driver's view. */
    private Translation2d currentGoal() {
        if (BLUE_ALLIANCE) {
            return rightGoal ? goalPosBS : goalPosBA;
        }
        return rightGoal ? goalPosRS : goalPosRA;
    }

    // ------------------------------------------------------------------
    // Points and fit
    // ------------------------------------------------------------------

    private void recordPoint() {
        if (mode == Mode.FIT) {
            message = "Switch to MANUAL (Y) to record. FIT mode is only for testing the fit.";
            return;
        }
        if (!localized) {
            message = "Not recorded: relocalize first with gamepad 2 A/B/X/Y. The robot starts at "
                    + "field center, so the distance is wrong until then.";
            return;
        }

        ShotPoint near = null;
        for (ShotPoint p : points) {
            if (Math.abs(p.distance - distance) < NEAR_INCHES) near = p;
        }

        points.add(new ShotPoint(distance, manualVelocity, manualHood));
        RobotLog.ii("ShooterDistanceTune",
                "point %d: distance %.1f in, velocity %.0f tps, hood %.2f, pose (%.1f, %.1f, %.0f deg), goal %s %s",
                points.size(), distance, manualVelocity, manualHood,
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()),
                BLUE_ALLIANCE ? "blue" : "red", rightGoal ? "T1" : "T2");
        fit();

        message = String.format(Locale.US, "Recorded point %d at %.1f in.", points.size(), distance);
        if (near != null) {
            message += String.format(Locale.US,
                    " Only %.1f in from an earlier point; both are kept. Press B if that was a double press.",
                    Math.abs(near.distance - distance));
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
        multipleTelemetry.addData("Odometry", localized ? "relocalized at " + resetName
                : "NOT RELOCALIZED: gamepad 2 A/B/X/Y");
        multipleTelemetry.addData("Goal", "%s %s", BLUE_ALLIANCE ? "blue" : "red",
                rightGoal ? "right (T1)" : "left (T2)");
        multipleTelemetry.addData("Pose", "x %.1f  y %.1f  heading %.0f deg",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

        // Plain numbers so they can be graphed in FTC Dashboard
        multipleTelemetry.addData("Distance (in)", distance);
        multipleTelemetry.addData("Raw distance to goal (in)", goalDistance);
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
        final double distance;  // inches, after DISTANCE_OFFSET_INCHES, same as the match code
        final double velocity;  // ticks/sec
        final double hood;      // same units as setHoodAngle()

        ShotPoint(double distance, double velocity, double hood) {
            this.distance = distance;
            this.velocity = velocity;
            this.hood = hood;
        }
    }
}