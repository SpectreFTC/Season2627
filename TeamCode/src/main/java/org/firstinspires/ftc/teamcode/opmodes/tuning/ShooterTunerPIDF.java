package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.apache.commons.collections4.queue.CircularFifoQueue;
import org.firstinspires.ftc.teamcode.globals.BulkCache;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

import java.util.Locale;

/**
 * Manual shooter tuner using SolversLib's {@link PIDFController}. Edit the gains in FTC
 * Dashboard while the flywheel runs and watch the graph respond.
 *
 * <h2>The controller</h2>
 * <p>Same shooter setup as ShooterSubsystem: both motors in {@code RUN_WITHOUT_ENCODER},
 * velocity read from {@code shooterMotor1} and negated, the same power sent to both
 * motors. The gains are in volts, and the total is divided by battery voltage to get a
 * power, so tuned gains work the same on a fresh or worn battery.</p>
 * <pre>
 *     pidfVolts = kP * error + kI * integral + kD * d(error)/dt + kF * target   (SolversLib)
 *     volts     = pidfVolts + kS                                                (added here)
 *     power     = volts / batteryVoltage
 * </pre>
 * <ul>
 *   <li><b>kF</b> does the same job as kV from ShooterTunerStep1 (volts per tick/sec of
 *       target), and starts at that value.</li>
 *   <li><b>kS</b> is added outside the controller. SolversLib has no static friction term;
 *       its {@code setOpenF} flips sign with the error, which would push the flywheel
 *       backwards every time it went past the target.</li>
 *   <li><b>MAX_INTEGRAL_VOLTS</b> is converted to SolversLib's integration bounds
 *       ({@code MAX_INTEGRAL_VOLTS / kI}). SolversLib's default bounds are +/-1, which
 *       makes kI do almost nothing on a flywheel.</li>
 *   <li><b>I_ZONE_TPS</b> clears the integral while the error is larger than this, so it
 *       can't build up during spin-up or a shot and push the speed past the target.</li>
 *   <li><b>DECAY_FACTOR</b> is SolversLib's integral decay: each loop the integral points
 *       the opposite way from the error, it is multiplied by this. 1 turns it off.</li>
 * </ul>
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li>The flywheel spins on START. Gamepad 1 A pauses and resumes.</li>
 *   <li>{@code ALTERNATE = true} switches between LOW_VELOCITY and HIGH_VELOCITY every
 *       STEP_SECONDS, so you see a fresh step response every few seconds.</li>
 *   <li>{@code ALTERNATE = false} holds HIGH_VELOCITY. Use it to feed game pieces and
 *       watch the dip and recovery after each shot.</li>
 *   <li>Graph Target, Velocity and the volts terms in FTC Dashboard. Each step's overshoot,
 *       settle time and average error show in telemetry.</li>
 * </ul>
 *
 * <h2>Tuning order</h2>
 * <ol>
 *   <li><b>Feedforward only</b> (kP = kI = kD = 0). The speed should creep to within a few
 *       percent of the target. If it lands off by about the same tps at both speeds, adjust
 *       kS; if it's off by the same percentage, adjust kF.</li>
 *   <li><b>kP.</b> Start around the size of kF and double it until the speed overshoots,
 *       then back off about a third. kP is what makes spin-up and shot recovery fast.</li>
 *   <li><b>kI</b>, only if the speed settles off target. Keep it small. If it adds
 *       overshoot, lower kI or I_ZONE_TPS, or try DECAY_FACTOR around 0.9.</li>
 *   <li><b>kD</b> is usually left at 0 for a flywheel. Velocity readings are noisy and D
 *       amplifies the noise. SolversLib's D also spikes for one loop each time the target
 *       changes.</li>
 * </ol>
 * <p>Dashboard edits only last until the app restarts. Copy the final numbers into
 * RobotConstants.java to keep them.</p>
 */
@Disabled
@Config
@TeleOp
public class ShooterTunerPIDF extends OpMode {

    // ---- SolversLib PIDF gains, in volts (editable from FTC Dashboard) ----
    public static double kP = RobotConstants.SHOOTER_kP;  // volts per tick/sec of error
    public static double kI = RobotConstants.SHOOTER_kI;  // volts per (tick/sec * s) of built-up error
    public static double kD = 0;                          // volts per tick/sec^2 of error change
    public static double kF = RobotConstants.kV;          // volts per tick/sec of target, same as kV
    public static double kS = RobotConstants.kS;          // volts to overcome friction, added outside the PIDF

    //Intake and Transfer Powers
    public static double intakePower = 1.0;
    public static double transferPower = -1.0;

    //Hood
    public static double hoodAngle = 0.8;

    // ---- Integral limits ----
    public static double I_ZONE_TPS = RobotConstants.SHOOTER_I_ZONE;  // integral cleared beyond this error; 0 = off
    public static double MAX_INTEGRAL_VOLTS = RobotConstants.SHOOTER_MAX_INTEGRAL_VOLTS;  // most volts kI may add
    public static double DECAY_FACTOR = 1.0;  // 1 = off; below 1 shrinks an integral pointing the wrong way

    // ---- Target ----
    public static boolean ALTERNATE = false;     // true: LOW <-> HIGH every STEP_SECONDS; false: hold HIGH
    public static double LOW_VELOCITY = 735;    // ticks/sec
    public static double HIGH_VELOCITY = 735;  // ticks/sec
    public static double STEP_SECONDS = 4.0;

    public static double VOLTAGE_READ_MS = 50;

    // "Settled" means within 5% of the step size or this many tps, whichever is bigger
    private static final double MIN_SETTLE_BAND_TPS = 15;
    private static final int ERROR_AVERAGE_LOOPS = 25;

    RobotHardware robotHardware;
    VoltageSensor voltageSensor;
    MultipleTelemetry multipleTelemetry;
    BulkCache bulkCache;
    PIDFController controller;
    ElapsedTime loopTimer;
    ElapsedTime voltageTimer;
    ElapsedTime alternateTimer;
    ElapsedTime stepTimer;
    RollingAverage errorAverage;

    private boolean running = true;
    private boolean lastA = false;
    private boolean highTarget = false;  // which target ALTERNATE is on; starts at LOW
    private double batteryVoltage = 12;

    // ---- Last loop, for telemetry ----
    private double target = 0;
    private double velocity = 0;
    private double error = 0;
    private double power = 0;
    private double fVolts, sVolts, pVolts, iVolts, dVolts;

    // ---- Current step ----
    private double stepTarget = Double.NaN;
    private double stepFrom, stepPeak, stepLastOutside;
    private boolean stepInBand = false;

    // ---- Last finished step ----
    private boolean haveLastStep = false;
    private double lastFrom, lastTarget, lastOvershoot, lastSettle, lastAvgError;
    private boolean lastSettled;

    @Override
    public void init() {
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware = RobotHardware.getInstance(this);
        robotHardware.shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardware.shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardware.shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardware.shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // After RobotHardware, so its bulk caching mode isn't overwritten
        bulkCache = new BulkCache(hardwareMap);

        voltageSensor = robotHardware.voltageSensor;
        batteryVoltage = voltageSensor.getVoltage();

        controller = new PIDFController(kP, kI, kD, kF);

        loopTimer = new ElapsedTime();
        voltageTimer = new ElapsedTime();
        alternateTimer = new ElapsedTime();
        stepTimer = new ElapsedTime();
        errorAverage = new RollingAverage(ERROR_AVERAGE_LOOPS);

        multipleTelemetry.addLine("Shooter PIDF tuner (SolversLib). Edit gains in FTC Dashboard.");
        multipleTelemetry.addLine("The shooter spins up on START. Gamepad 1 A pauses and resumes.");
        multipleTelemetry.addLine("Keep hands clear.");
        multipleTelemetry.update();
    }

    @Override
    public void start() {
        running = true;
        highTarget = false;
        controller.reset();
        loopTimer.reset();
        voltageTimer.reset();
        alternateTimer.reset();
    }

    @Override
    public void loop() {
        // Fresh bulk data every loop; without this, the cached velocity would never update
        bulkCache.clearCache();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // A pauses and resumes. Edge-detected so holding it doesn't toggle every loop
        boolean a = gamepad1.a;
        if (a && !lastA) {
            running = !running;
            controller.reset();  // so the pause isn't counted as one long loop in I and D
            alternateTimer.reset();
        }
        lastA = a;

        // Battery voltage isn't in the bulk data, and each read is a separate hub request
        if (voltageTimer.milliseconds() >= VOLTAGE_READ_MS) {
            batteryVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        // Same sign as ShooterSubsystem and the other tuners
        velocity = -robotHardware.shooterMotor1.getVelocity();

        if (ALTERNATE && alternateTimer.seconds() >= STEP_SECONDS) {
            highTarget = !highTarget;
            alternateTimer.reset();
        }
        double requested = (ALTERNATE && !highTarget) ? LOW_VELOCITY : HIGH_VELOCITY;
        target = running ? requested : 0;

        if (target != stepTarget) {
            beginStep();
        }
        updateStep();

        if (target > 0) {
            runController();
        } else {
            // No kS at zero, or the flywheel would creep. Coasts down like ShooterSubsystem.stop()
            controller.reset();
            error = 0;
            power = 0;
            fVolts = sVolts = pVolts = iVolts = dVolts = 0;
        }

        //Intake/Transfer
        if (gamepad1.b) {
            robotHardware.intakeMotor.setPower(intakePower);
            robotHardware.transferMotor.setPower(transferPower);
        }
        else {
            robotHardware.intakeMotor.setPower(0);
            robotHardware.transferMotor.setPower(0);
        }

        robotHardware.hoodServo.setPosition(hoodAngle);

        setShooterPower(power);


        sendTelemetry(loopMs);
    }

    @Override
    public void stop() {
        setShooterPower(0);
    }

    // ------------------------------------------------------------------
    // Controller
    // ------------------------------------------------------------------

    private void runController() {
        error = target - velocity;

        // Picks up gain changes made in FTC Dashboard while running
        controller.setPIDF(kP, kI, kD, kF);

        // SolversLib bounds the integral sum (tps * s), not volts, so convert
        double bound = (kI > 0 && MAX_INTEGRAL_VOLTS > 0) ? MAX_INTEGRAL_VOLTS / kI : 0;
        controller.integrationControl.setIntegrationBounds(-bound, bound);
        controller.integrationControl.setDecayFactor(Range.clip(DECAY_FACTOR, 0, 1));

        // I-zone: no integral build-up during spin-up or a shot
        if (I_ZONE_TPS > 0 && Math.abs(error) > I_ZONE_TPS) {
            controller.clearTotalError();
        }

        double pidfVolts = controller.calculate(velocity, target);
        sVolts = Math.signum(target) * kS;

        // Split into terms for the graph. calculate() is kP*e + kI*sum + kD*de/dt + kF*target
        fVolts = kF * target;
        pVolts = kP * controller.getPositionError();
        dVolts = kD * controller.getVelocityError();
        iVolts = pidfVolts - fVolts - pVolts - dVolts;

        power = Range.clip((pidfVolts + sVolts) / batteryVoltage, -1, 1);
        errorAverage.add(error);
    }

    private void setShooterPower(double p) {
        robotHardware.shooterMotor1.setPower(p);
        robotHardware.shooterMotor2.setPower(p);
    }

    // ------------------------------------------------------------------
    // Step measurements
    // ------------------------------------------------------------------

    /** Called when the target changes. Saves the step that just ended and starts a new one. */
    private void beginStep() {
        if (stepTarget > 0 && Math.abs(stepTarget - stepFrom) >= 1) {
            haveLastStep = true;
            lastFrom = stepFrom;
            lastTarget = stepTarget;
            lastOvershoot = overshootPercent();
            lastSettled = stepInBand;
            lastSettle = stepLastOutside;
            lastAvgError = errorAverage.mean();
        }
        stepTarget = target;
        stepFrom = velocity;
        stepPeak = velocity;
        stepLastOutside = 0;
        stepInBand = false;
        stepTimer.reset();
        errorAverage.clear();
    }

    private void updateStep() {
        if (stepTarget <= 0) return;
        double change = stepTarget - stepFrom;
        double direction = Math.signum(change);
        if ((velocity - stepPeak) * direction > 0) stepPeak = velocity;

        double band = Math.max(0.05 * Math.abs(change), MIN_SETTLE_BAND_TPS);
        stepInBand = Math.abs(velocity - stepTarget) <= band;
        if (!stepInBand) stepLastOutside = stepTimer.seconds();
    }

    /** How far the speed went past the target, as a percent of the step size. */
    private double overshootPercent() {
        double change = stepTarget - stepFrom;
        if (Math.abs(change) < 1) return 0;
        return Math.max(0, (stepPeak - stepTarget) * Math.signum(change)) / Math.abs(change) * 100;
    }

    // ------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------

    private void sendTelemetry(double loopMs) {
        multipleTelemetry.addData("State", running ? "RUNNING (A to pause)" : "PAUSED (A to resume)");

        // Plain numbers so they can be graphed in FTC Dashboard
        multipleTelemetry.addData("Target (tps)", target);
        multipleTelemetry.addData("Velocity (tps)", velocity);
        multipleTelemetry.addData("Error (tps)", error);
        multipleTelemetry.addData("Power", power);
        multipleTelemetry.addData("F (V)", fVolts);
        multipleTelemetry.addData("S (V)", sVolts);
        multipleTelemetry.addData("P (V)", pVolts);
        multipleTelemetry.addData("I (V)", iVolts);
        multipleTelemetry.addData("D (V)", dVolts);
        multipleTelemetry.addData("Battery (V)", batteryVoltage);
        multipleTelemetry.addData("Loop (ms)", loopMs);

        if (stepTarget > 0) {
            multipleTelemetry.addLine();
            multipleTelemetry.addLine(String.format(Locale.US, "This step: %.0f -> %.0f tps", stepFrom, stepTarget));
            multipleTelemetry.addLine(String.format(Locale.US, "  Overshoot %.1f%%, %s, avg error %s",
                    overshootPercent(),
                    stepInBand ? String.format(Locale.US, "settled after %.2f s", stepLastOutside) : "not settled",
                    formatError(errorAverage.mean())));
        }
        if (haveLastStep) {
            multipleTelemetry.addLine(String.format(Locale.US, "Last step: %.0f -> %.0f tps", lastFrom, lastTarget));
            multipleTelemetry.addLine(String.format(Locale.US, "  Overshoot %.1f%%, %s, avg error %s",
                    lastOvershoot,
                    lastSettled ? String.format(Locale.US, "settled after %.2f s", lastSettle) : "never settled",
                    formatError(lastAvgError)));
        }

        multipleTelemetry.addLine();
        multipleTelemetry.addLine(String.format(Locale.US,
                "kP %.6g  kI %.6g  kD %.6g  kF %.6g  kS %.4g", kP, kI, kD, kF, kS));
        multipleTelemetry.addLine(String.format(Locale.US,
                "I zone %.0f tps  max I %.2f V  decay %.2f", I_ZONE_TPS, MAX_INTEGRAL_VOLTS, DECAY_FACTOR));

        multipleTelemetry.update();
    }

    private static String formatError(double value) {
        return Double.isNaN(value) ? "-" : String.format(Locale.US, "%.1f tps", value);
    }

    // ------------------------------------------------------------------
    // Rolling average
    // ------------------------------------------------------------------

    /**
     * Average of the most recent values, up to a fixed count. CircularFifoQueue plus a
     * running sum, so adding a value and getting the mean are both instant.
     */
    static class RollingAverage {
        private final CircularFifoQueue<Double> window;
        private double sum = 0;

        RollingAverage(int capacity) {
            window = new CircularFifoQueue<>(Math.max(capacity, 1));
        }

        void add(double value) {
            if (window.isAtFullCapacity()) {
                sum -= window.remove();  // oldest value; remove() never returns null
            }
            window.add(value);
            sum += value;
        }

        void clear() {
            window.clear();
            sum = 0;
        }

        double mean() {
            return window.isEmpty() ? Double.NaN : sum / window.size();
        }
    }
}