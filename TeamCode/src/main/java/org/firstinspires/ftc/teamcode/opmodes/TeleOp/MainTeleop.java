package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import static org.firstinspires.ftc.teamcode.globals.RobotConstants.AIM_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.ClusterBearing;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.globals.BulkCache;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;
import org.firstinspires.ftc.teamcode.globals.RobotUtils;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

public class MainTeleop {
    public final RobotHardware robotHardware;
    public final GamepadEx gamepad1;
    public final GamepadEx gamepad2;
    public final OpMode opMode;
    public final List<Subsystem> subsystems;
    public final BulkCache bulkCache;
    public final ShooterSubsystem shooterSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final Follower follower;
    public final MultipleTelemetry multipleTelemetry;
    public final ClusterBearing aim;
    int intakeState = 1;
    boolean flowerMechDown = false;
    boolean isFlyWheelStopped = false;
    boolean isAiming;
    //the robot should always be facing towards you when relocalizing in the opposite corners,
    //and in the adjacent corners the robot is facing way, so you could that the back of the robot is always facing a driver wall
    public MainTeleop(Gamepad gamepad1, Gamepad gamepad2,
                      OpMode opMode,
                      ShooterSubsystem shooterSubsystem,
                      IntakeSubsystem intakeSubsystem,
                      boolean isBlue
    ){
       this.gamepad1 = new GamepadEx(gamepad1);
       this.gamepad2 = new GamepadEx(gamepad2);
       this.opMode = opMode;
       robotHardware = RobotHardware.getInstance(opMode);
       bulkCache = new BulkCache(opMode.hardwareMap);
       this.subsystems = new ArrayList<>();
       this.shooterSubsystem = shooterSubsystem;
       this.intakeSubsystem = intakeSubsystem;
       subsystems.add(this.shooterSubsystem);
       subsystems.add(this.intakeSubsystem);
       multipleTelemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
       follower = Constants.create(opMode.hardwareMap);
       ClusterBearing.ALLIANCE = isBlue ? "BLUE" : "RED";
       aim = new ClusterBearing(opMode.hardwareMap, "Webcam");
    }
    private final ElapsedTime loopTimer = new ElapsedTime();

    public static double scale(double v){
        return Math.signum(v)*Math.sqrt(Math.abs(v));
    }

    public void loop(){
        bulkCache.clearCache();
        follower.update();
        gamepad1.readButtons();
        gamepad2.readButtons();
        isAiming = gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;
        double turn = -0.6 * gamepad1.getRightX();
        if (isAiming) {
            double b = aim.getBearing();
            if (!Double.isNaN(b)) turn = Range.clip(AIM_KP * b, -1, 1);
        }
        follower.manual(gamepad1.getLeftY(), -gamepad1.getLeftX(), turn);

        // 0 = intake-ing, 1 = holding, 2 = shooting, 3 = ejecting
        if (gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            intakeState = 0;
        } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            intakeState = 3;
        } else if (gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            intakeState = 2;
        } else {
            intakeState = 1;
        }

        switch (intakeState) {
            case 0: intakeSubsystem.intake(); break;
            case 1: intakeSubsystem.hold(); break;
            case 2: intakeSubsystem.shoot(); break;
            case 3: intakeSubsystem.eject(); break;
        }

        if (gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            flowerMechDown = !flowerMechDown;
        }
        // After the switch, same as before, so it wins if they share motors
        if (flowerMechDown) {
            intakeSubsystem.flowerIntakeRampDown();
        } else {
            intakeSubsystem.flowerIntakeRampUp();
        }

        if(gamepad1.wasJustPressed(GamepadKeys.Button.A)){
            isFlyWheelStopped = !isFlyWheelStopped;
        }

        shooterSubsystem.runAtVelocity(RobotConstants.setVelocity, robotHardware.voltageSensor::getVoltage);
        shooterSubsystem.setHoodAngle(RobotConstants.setHoodAngle);

        shooterSubsystem.update();
        intakeSubsystem.update();

        multipleTelemetry.addData("Loop (ms)", loopTimer.milliseconds());
        multipleTelemetry.addData("velo", shooterSubsystem.getVelocity());
        multipleTelemetry.addData("hood", robotHardware.hoodServo.getPosition());
        multipleTelemetry.addData("lastFix", aim.getBearing());
        multipleTelemetry.addData("aiming at ", aim.getTargetName());
        multipleTelemetry.update();
        loopTimer.reset();
    }

    public void init(){
        for (Subsystem subsystem : subsystems) {
            subsystem.init();
        }

    }
}
