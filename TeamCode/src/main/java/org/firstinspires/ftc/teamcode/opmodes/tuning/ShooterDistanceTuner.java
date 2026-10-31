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

import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.PinpointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.globals.BulkCache;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;
import org.firstinspires.ftc.teamcode.globals.ShotTable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

/**
 * Manual shot-table tuner. At each distance you dial in the flywheel velocity and hood angle
 * by hand until shots go in, and record the point. The points become {@link ShotTable}, which
 * interpolates between them with SolversLib's InterpLUT (a smooth curve through every point).
 *
 * <p>The distance comes from Pinpoint odometry, calculated the same way as MainTeleop:
 * {@code getDistanceTo(goal) - 22}. The robot starts at {@code new Pose2d()} (field center),
 * so relocalize at a corner before recording. The shooter and intake run through
 * ShooterSubsystem and IntakeSubsystem with MainTeleop's buttons, so shots are fed the way
 * they are in matches.</p>
 *
 * <h2>How to use it</h2>
 * <ol>
 *   <li>Set {@code BLUE_ALLIANCE} in FTC Dashboard. Push the robot into a corner and press that
 *       corner's button on gamepad 2 (A, B, X or Y) to relocalize. Pick the goal with gamepad 2
 *       dpad right/left, the same as MainTeleop.</li>
 *   <li>Drive to a shooting spot. Adjust velocity (dpad up/down) and hood angle (dpad
 *       left/right) and shoot (right trigger) until shots go in reliably. Shoot only when
 *       telemetry says READY.</li>
 *   <li>Press X to record the point. Recording within {@code REPLACE_WITHIN_INCHES} of an
 *       existing point replaces it, so you can re-tune one distance without starting over.</li>
 *   <li>Repeat across the whole range. The table holds its end values past the closest and
 *       farthest points, so record a little beyond where you'll shoot from. More points where
 *       the right velocity changes quickly; about every 12 to 18 inches is a good start.</li>
 *   <li>Press Y for TABLE mode and drive around: the shooter follows the table using the live
 *       distance, the same as in a match. Check distances between your points.</li>
 *   <li>Copy the three arrays from telemetry into ShotTable.java to keep them.</li>
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
 *   <li>A: flywheel on/off. X: record a point. B: delete the point recorded most recently.
 *       Y: switch between MANUAL and TABLE.</li>
 * </ul>
 * <p>Gamepad 2:</p>
 * <ul>
 *   <li>A, B, X, Y: relocalize at RA, RS, BA, BS (the same poses as MainTeleop's gamepad 1
 *       A, B, X, Y).</li>
 *   <li>Dpad right/left: goal on the right (T1) or left (T2), from the driver's view.</li>
 * </ul>
 *
 * <h2>Points and the table</h2>
 * <p>The first run after the app starts loads the points already in ShotTable.java, so you can
 * refine an existing table. Points are kept until the app restarts, so stopping and restarting
 * this OpMode doesn't lose them. With 2 or more points, the table is applied to ShotTable for
 * this app session, so MainTeleop uses it right away. Set {@code CLEAR_POINTS} to true in
 * Dashboard to start over. Every change is also written to the robot log.</p>
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
    public static double REPLACE_WITHIN_INCHES = 3;  // recording this close to a point replaces it
    public static double READY_TOLERANCE_TPS = 30;   // READY when the speed is this close to the target
    public static double DRIVE_SPEED = 0.5;      // stick scale, so small aiming moves are easy
    public static boolean CLEAR_POINTS = false;  // set true to delete every point

    // Shooter distance = distance to the goal minus this. Must match the "- 22" in
    // MainTeleop's distance supplier, or the table will be shifted by 22 inches.
    private static final double DISTANCE_OFFSET_INCHES = 22;

    // Field positions. These MUST be the same values as MainTeleop's, in the same field frame.
    private static final Pose2d RESET_RA = new Pose2d(63.182, 69.812, Rotation2d.fromDegrees(-90));
    private static final Pose2d RESET_RS = new Pose2d(63.182, -69.812, Rotation2d.fromDegrees(-90));
    private static final Pose2d RESET_BA = new Pose2d(-63.182, -69.812, Rotation2d.fromDegrees(90));
    private static final Pose2d RESET_BS = new Pose2d(-63.182, 69.812, Rotation2d.fromDegrees(90));
    private static final Translation2d GOAL_B1 = new Translation2d(-12.758, -14.19);
    private static final Translation2d GOAL_B2 = new Translation2d(-12.758, 14.19);
    private static final Translation2d GOAL_R1 = new Translation2d(12.742, -14.19);
    private static final Translation2d GOAL_R2 = new Translation2d(12.742, 14.19);

    // Kept across OpMode runs until the app restarts. In recording order: the last one is the
    // most recent, which is what B deletes.
    private static final List<ShotPoint> points = new ArrayList<>();
    private static boolean loadedFileTable = false;

    private enum Mode { MANUAL, TABLE }

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

    // Problem found when applying the table, shown until the next change
    private String tableError = null;

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

        // First run since the app started: begin from the table in ShotTable.java
        if (!loadedFileTable) {
            loadedFileTable = true;
            if (points.isEmpty()) {
                for (int i = 0; i < ShotTable.DISTANCES.length; i++) {
                    points.add(new ShotPoint(ShotTable.DISTANCES[i], ShotTable.VELOCITIES[i], ShotTable.HOOD_ANGLES[i]));
                }
            }
        }
        applyTable();

        multipleTelemetry.addLine("Shooter distance tuner. The flywheel spins up on START (A toggles it).");
        multipleTelemetry.addLine("Relocalize with gamepad 2 A/B/X/Y before recording.");
        multipleTelemetry.addData("Points", points.size());
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
            mode = mode == Mode.MANUAL ? Mode.TABLE : Mode.MANUAL;
            message = mode == Mode.TABLE
                    ? (ShotTable.hasTable() ? "TABLE: using the shot table." : "TABLE: " + ShotTable.describe() + ".")
                    : "MANUAL: using your velocity and hood.";
        }
        if (x) recordPoint();
        if (b) deleteLastPoint();
        if (CLEAR_POINTS) {
            CLEAR_POINTS = false;
            points.clear();
            applyTable();
            message = "Cleared all points.";
        }

        // ---- Shooter ----
        if (mode == Mode.TABLE) {
            // Same lookup as ShooterSubsystem.shooterSetVelocityRunCommand(), with the live distance
            targetVelocity = ShotTable.velocity(distance);
            targetHood = ShotTable.hoodAngle(distance);
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
            return rightGoal ? GOAL_B1 : GOAL_B2;
        }
        return rightGoal ? GOAL_R1 : GOAL_R2;
    }

    // ------------------------------------------------------------------
    // Points and table
    // ------------------------------------------------------------------

    private void recordPoint() {
        if (mode == Mode.TABLE) {
            message = "Switch to MANUAL (Y) to record. TABLE mode is only for testing the table.";
            return;
        }
        if (!localized) {
            message = "Not recorded: relocalize first with gamepad 2 A/B/X/Y. The robot starts at "
                    + "field center, so the distance is wrong until then.";
            return;
        }

        // InterpLUT needs every distance to be different, and two close points with different
        // values make the curve swing between them, so a nearby point is replaced
        ShotPoint replaced = null;
        for (ShotPoint p : points) {
            if (Math.abs(p.distance - distance) < REPLACE_WITHIN_INCHES
                    && (replaced == null || Math.abs(p.distance - distance) < Math.abs(replaced.distance - distance))) {
                replaced = p;
            }
        }
        if (replaced != null) points.remove(replaced);

        points.add(new ShotPoint(distance, manualVelocity, manualHood));
        RobotLog.ii("ShooterDistanceTune",
                "point at %.1f in: velocity %.0f tps, hood %.2f, pose (%.1f, %.1f, %.0f deg), goal %s %s",
                distance, manualVelocity, manualHood,
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()),
                BLUE_ALLIANCE ? "blue" : "red", rightGoal ? "T1" : "T2");
        applyTable();

        message = replaced == null
                ? String.format(Locale.US, "Recorded a point at %.1f in.", distance)
                : String.format(Locale.US, "Replaced the point at %.1f in with one at %.1f in.",
                replaced.distance, distance);
    }

    private void deleteLastPoint() {
        if (points.isEmpty()) {
            message = "No points to delete.";
            return;
        }
        ShotPoint removed = points.remove(points.size() - 1);
        applyTable();
        message = String.format(Locale.US, "Deleted the point at %.1f in.", removed.distance);
    }

    /**
     * Applies the points to ShotTable for this app session, so TABLE mode and the match code
     * use them right away. With fewer than 2 points, ShotTable goes back to ShotTable.java.
     */
    private void applyTable() {
        tableError = null;
        List<ShotPoint> sorted = sortedPoints();
        try {
            if (sorted.size() >= 2) {
                double[][] columns = columns(sorted);
                ShotTable.set(columns[0], columns[1], columns[2]);
            } else {
                ShotTable.reset();
            }
        } catch (IllegalArgumentException e) {
            // Shouldn't happen, since close points are replaced, but don't crash the OpMode
            tableError = e.getMessage();
            ShotTable.reset();
        }
        RobotLog.ii("ShooterDistanceTune", "table: %s", tableArrays(sorted).replace('\n', ' '));
    }

    private static List<ShotPoint> sortedPoints() {
        List<ShotPoint> sorted = new ArrayList<>(points);
        Collections.sort(sorted, (p, q) -> Double.compare(p.distance, q.distance));
        return sorted;
    }

    private static double[][] columns(List<ShotPoint> sorted) {
        double[] d = new double[sorted.size()];
        double[] v = new double[sorted.size()];
        double[] h = new double[sorted.size()];
        for (int i = 0; i < sorted.size(); i++) {
            d[i] = sorted.get(i).distance;
            v[i] = sorted.get(i).velocity;
            h[i] = sorted.get(i).hood;
        }
        return new double[][]{d, v, h};
    }

    /** The three array lines to paste over the ones in ShotTable.java. */
    private static String tableArrays(List<ShotPoint> sorted) {
        StringBuilder d = new StringBuilder("public static final double[] DISTANCES = {");
        StringBuilder v = new StringBuilder("public static final double[] VELOCITIES = {");
        StringBuilder h = new StringBuilder("public static final double[] HOOD_ANGLES = {");
        for (int i = 0; i < sorted.size(); i++) {
            String sep = i == 0 ? "" : ", ";
            ShotPoint p = sorted.get(i);
            d.append(sep).append(String.format(Locale.US, "%.1f", p.distance));
            v.append(sep).append(String.format(Locale.US, "%.0f", p.velocity));
            h.append(sep).append(String.format(Locale.US, "%.2f", p.hood));
        }
        return d.append("};\n").append(v).append("};\n").append(h).append("};").toString();
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

        multipleTelemetry.addData("Mode", mode == Mode.MANUAL ? "MANUAL (Y for TABLE)" : "TABLE (Y for MANUAL)");
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

        if (mode == Mode.TABLE && ShotTable.isOutside(distance)) {
            multipleTelemetry.addLine(String.format(Locale.US,
                    "Outside the table (%.1f to %.1f in): holding the nearest end point's values.",
                    ShotTable.minDistance(), ShotTable.maxDistance()));
        }
        if (!message.isEmpty()) {
            multipleTelemetry.addLine();
            multipleTelemetry.addLine(message);
        }
        if (tableError != null) {
            multipleTelemetry.addLine("TABLE NOT APPLIED: " + tableError);
        }

        List<ShotPoint> sorted = sortedPoints();
        ShotPoint last = points.isEmpty() ? null : points.get(points.size() - 1);
        multipleTelemetry.addLine();
        multipleTelemetry.addLine(String.format(Locale.US, "Points (%d), by distance:", sorted.size()));
        for (int i = 0; i < sorted.size(); i++) {
            ShotPoint p = sorted.get(i);
            String line = String.format(Locale.US, "  %.1f in: %.0f tps, hood %.2f", p.distance, p.velocity, p.hood);
            // Farther shots almost always need more speed; a drop usually means a bad point
            if (i > 0 && p.velocity < sorted.get(i - 1).velocity) line += "  (slower than the closer point: check it)";
            if (p == last) line += "  (last; B deletes)";
            multipleTelemetry.addLine(line);
        }

        multipleTelemetry.addLine();
        multipleTelemetry.addData("Table", ShotTable.describe());
        if (sorted.size() >= 2) {
            multipleTelemetry.addLine("Applied for this session. Paste over the three lines in ShotTable.java:");
            for (String line : tableArrays(sorted).split("\n")) {
                multipleTelemetry.addLine(line);
            }
        } else {
            multipleTelemetry.addLine("Record at least 2 distances to make a table.");
        }

        multipleTelemetry.update();
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