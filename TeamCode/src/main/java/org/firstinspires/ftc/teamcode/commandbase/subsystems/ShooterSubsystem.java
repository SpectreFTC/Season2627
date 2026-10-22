package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_DECAY_FACTOR;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_I_ZONE;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_MAX_INTEGRAL_VOLTS;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_kD;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_kI;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_kP;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.bHoodAngle;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.bVelocity;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.hoodServoAngleAtCalibration;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.hoodServoPositionAtCalibration;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.hoodServoRange;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.kS;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.kV;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.mHoodAngle;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.mVelocity;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.globals.ServoEx;

import java.util.function.Supplier;

/**
 * Shooter flywheel (two motors) and hood.
 *
 * <p>Velocity control uses SolversLib's {@link PIDFController}, the same controller and
 * math as ShooterPIDFTuner, so gains tuned there behave the same here. Everything is in
 * volts and divided by battery voltage to get a power:</p>
 * <pre>
 *     pidfVolts = SHOOTER_kP * error + SHOOTER_kI * integral + SHOOTER_kD * d(error)/dt + kV * target
 *     power     = (pidfVolts + kS) / batteryVoltage
 * </pre>
 * <ul>
 *   <li><b>kV</b> is the PIDF's kF: volts per tick/sec of target (ShooterTunerStep1).</li>
 *   <li><b>kS</b> is added outside the PIDF. SolversLib has no static friction term, and
 *       its {@code setOpenF} flips sign with the error, which would push the flywheel
 *       backwards every time it went past the target.</li>
 *   <li><b>SHOOTER_kP</b> makes spin-up and shot recovery fast.</li>
 *   <li><b>SHOOTER_kI</b> trims leftover error. The integral is cleared while the error is
 *       bigger than {@code SHOOTER_I_ZONE}, so it can't build up during spin-up or a shot,
 *       and it can't add more than {@code SHOOTER_MAX_INTEGRAL_VOLTS}.</li>
 *   <li><b>SHOOTER_kD</b> is usually 0 for a flywheel; velocity readings are noisy.</li>
 *   <li><b>SHOOTER_DECAY_FACTOR</b> is SolversLib's integral decay. 1 turns it off.</li>
 * </ul>
 * <p>The gains are read from RobotConstants on every call, so changes made in FTC
 * Dashboard or by the tuners apply right away.</p>
 */
public class ShooterSubsystem implements Subsystem {
    private static final double VOLTAGE_READ_MS = 50;
    // Longer than this between calls and the controller starts fresh. Otherwise SolversLib
    // would count the whole gap as one loop, which kicks the integral and derivative.
    private static final double MAX_GAP_SECONDS = 0.1;

    private final ServoEx hoodServo;
    private final DcMotorEx shooterMotor1, shooterMotor2;
    private final PIDFController velocityController =
            new PIDFController(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, kV);

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime callTimer = new ElapsedTime();
    private double batteryVoltage = 12;
    private boolean haveVoltage = false;

    // Last values, for telemetry
    private double lastVelocity = 0;
    private double lastTargetVelocity = 0;
    private double lastPower = 0;
    private double lastIntegralVolts = 0;

    public ShooterSubsystem(Servo hoodServo, DcMotorEx shooterMotor1, DcMotorEx shooterMotor2) {
        this.hoodServo = new ServoEx(
                hoodServo,
                hoodServoRange,
                hoodServoPositionAtCalibration,
                hoodServoAngleAtCalibration,
                false
        );
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        this.shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * @param distanceSupplier supplier for distance (inches)
     * @param voltageSupplier  battery voltage in volts (only called every 50 ms)
     * @return a RunCommand that runs the shooter and hood for the given distance
     */
    public Command shooterSetVelocityRunCommand(Supplier<Double> distanceSupplier, Supplier<Double> voltageSupplier) {
        return new RunCommand(() -> {
            double distance = distanceSupplier.get();
            setHoodAngle(mHoodAngle * distance + bHoodAngle);
            runAtVelocity(mVelocity * distance + bVelocity, voltageSupplier);
        });
    }

    /** Stops the flywheel. Schedule this after interrupting the run command. */
    public Command stopCommand() {
        return new InstantCommand(this::stop);
    }

    /**
     * Holds the flywheel at targetVelocity (ticks/sec). Call every loop.
     * A target of 0 or less stops the flywheel.
     */
    public void runAtVelocity(double targetVelocity, Supplier<Double> voltageSupplier) {
        // Without this, a target of 0 would still send kS volts and the flywheel would creep
        if (targetVelocity <= 0) {
            stop();
            return;
        }

        // Battery voltage changes slowly, and each read is a separate hub request
        if (!haveVoltage || voltageTimer.milliseconds() >= VOLTAGE_READ_MS) {
            batteryVoltage = voltageSupplier.get();
            voltageTimer.reset();
            haveVoltage = true;
        }

        if (callTimer.seconds() > MAX_GAP_SECONDS) {
            velocityController.reset();
        }
        callTimer.reset();

        // Same sign as the tuners. Raw reading, no averaging: averaging adds delay
        double velocity = -shooterMotor1.getVelocity();
        double error = targetVelocity - velocity;

        // Picks up gain changes made in FTC Dashboard or by the tuners
        velocityController.setPIDF(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, kV);

        // SolversLib bounds the integral sum (tps * s), not volts, so convert.
        // Its default of +/-1 would make kI do almost nothing.
        double bound = (SHOOTER_kI > 0 && SHOOTER_MAX_INTEGRAL_VOLTS > 0)
                ? SHOOTER_MAX_INTEGRAL_VOLTS / SHOOTER_kI : 0;
        velocityController.integrationControl.setIntegrationBounds(-bound, bound);
        velocityController.integrationControl.setDecayFactor(Range.clip(SHOOTER_DECAY_FACTOR, 0, 1));

        // I-zone: no integral build-up during spin-up or a shot
        if (SHOOTER_I_ZONE > 0 && Math.abs(error) > SHOOTER_I_ZONE) {
            velocityController.clearTotalError();
        }

        double pidfVolts = velocityController.calculate(velocity, targetVelocity);
        double power = Range.clip((pidfVolts + kS) / batteryVoltage, -1, 1);

        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);

        // calculate() is kP*e + kI*sum + kD*de/dt + kF*target, so the rest is the integral
        lastIntegralVolts = pidfVolts
                - kV * targetVelocity
                - SHOOTER_kP * velocityController.getPositionError()
                - SHOOTER_kD * velocityController.getVelocityError();
        lastVelocity = velocity;
        lastTargetVelocity = targetVelocity;
        lastPower = power;
    }

    public void stop() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        velocityController.reset();  // so old integral doesn't carry into the next spin-up
        lastTargetVelocity = 0;
        lastPower = 0;
        lastIntegralVolts = 0;
    }

    public void setHoodAngle(double angle) {
        hoodServo.setAngle(angle);
    }

    public double getVelocity() {
        return lastVelocity;
    }

    public double getTargetVelocity() {
        return lastTargetVelocity;
    }

    public double getPower() {
        return lastPower;
    }

    /** Volts the integral is currently adding. Should stay small if the feedforward is good. */
    public double getIntegralVolts() {
        return lastIntegralVolts;
    }

    @Override
    public void update() {

    }

    @Override
    public void init() {

    }
}