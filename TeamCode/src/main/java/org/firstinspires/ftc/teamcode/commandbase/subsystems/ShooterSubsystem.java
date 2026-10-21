package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.SHOOTER_INTEGRAL_BOUND;
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
 * <p>Velocity control is feedforward plus PID, both in volts, divided by battery
 * voltage to get a power. This is the same structure ShooterTunerStep2 tunes for:</p>
 * <ul>
 *   <li>kV, kS: from ShooterTunerStep1 (volts needed to hold a speed).</li>
 *   <li>SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_INTEGRAL_BOUND: from
 *       ShooterTunerStep2 (volts of correction per tick/sec of error).</li>
 * </ul>
 */
public class ShooterSubsystem implements Subsystem {
    private static final double VOLTAGE_READ_MS = 50;

    private final ServoEx hoodServo;
    private final DcMotorEx shooterMotor1, shooterMotor2;
    private final PIDFController shooterVelocityController =
            new PIDFController(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, 0);

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double batteryVoltage = 12;
    private boolean haveVoltage = false;

    // Last values, for telemetry
    private double lastVelocity = 0;
    private double lastTargetVelocity = 0;
    private double lastPower = 0;

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
        // SolversLib clamps the integral to +/-1 by default, which makes kI do nothing
        shooterVelocityController.setIntegrationBounds(-SHOOTER_INTEGRAL_BOUND, SHOOTER_INTEGRAL_BOUND);
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

        // Same sign as ShooterTunerStep2. Raw reading, no averaging: averaging adds delay
        double velocity = -shooterMotor1.getVelocity();

        // Picks up gain changes made in FTC Dashboard while running
        shooterVelocityController.setPIDF(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, 0);
        shooterVelocityController.setIntegrationBounds(-SHOOTER_INTEGRAL_BOUND, SHOOTER_INTEGRAL_BOUND);

        double ffVolts = kV * targetVelocity + kS;                                       // hold the speed
        double pidVolts = shooterVelocityController.calculate(velocity, targetVelocity);  // fix the error
        double power = Range.clip((ffVolts + pidVolts) / batteryVoltage, -1, 1);

        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);

        lastVelocity = velocity;
        lastTargetVelocity = targetVelocity;
        lastPower = power;
    }

    public void stop() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        shooterVelocityController.reset();  // so old integral doesn't carry into the next spin-up
        lastTargetVelocity = 0;
        lastPower = 0;
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

    @Override
    public void update() {

    }

    @Override
    public void init() {

    }
}