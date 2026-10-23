package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.PinpointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.globals.BulkCache;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

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
    public final MecanumDriveSubsystem driveSubsystem;
    public final PinpointOdometrySubsystem pinpointOdometrySubsystem;
    public final CommandScheduler commandScheduler;
    Pose2d resetPoseRA = new Pose2d(63.182, 69.812, Rotation2d.fromDegrees(-90)),
            resetPoseRS = new Pose2d(63.182, -69.182, Rotation2d.fromDegrees(-90)),
            resetPoseBS = new Pose2d(-63.182, 69.812, Rotation2d.fromDegrees(90)),
            resetPoseBA = new Pose2d(-63.182, 69.812, Rotation2d.fromDegrees(90));
    Translation2d goalPos = new Translation2d(), goalPosT1, goalPosT2;
    Translation2d goalPosB1 = new Translation2d(-12.758, -14.19), goalPosB2 = new Translation2d(-12.758, 14.19);
    Translation2d goalPosR1 = new Translation2d(12.742, -14.19), goalPosR2 = new Translation2d(12.742,14.19);
    int intakeState = 1;
    boolean flowerMechDown = false;
    //the robot should always be facing towards you when relocalizing in the opposite corners,
    //and in the adjacent corners the robot is facing way, so you could that the back of the robot is always facing a driver wall
    public MainTeleop(Gamepad gamepad1, Gamepad gamepad2,
                      OpMode opMode,
                      ShooterSubsystem shooterSubsystem,
                      IntakeSubsystem intakeSubsystem,
                      MecanumDriveSubsystem driveSubsystem,
                      PinpointOdometrySubsystem pinpointOdometrySubsystem,
                      CommandScheduler commandScheduler,
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
       this.driveSubsystem = driveSubsystem;
       this.pinpointOdometrySubsystem = pinpointOdometrySubsystem;
       subsystems.add(this.shooterSubsystem);
       subsystems.add(this.intakeSubsystem);
       subsystems.add(this.driveSubsystem);
       subsystems.add(this.pinpointOdometrySubsystem);
       this.commandScheduler = commandScheduler;
       if(isBlue){
         goalPosT1 = goalPosB1;
         goalPosT2 = goalPosB2;
       }else{
         goalPosT1 = goalPosR1;
         goalPosT2 = goalPosR2;
       }
    }
    private final ElapsedTime loopTimer = new ElapsedTime();

    public static double scale(double v){
        return Math.signum(v)*Math.sqrt(Math.abs(v));
    }

    public void loop(){
        bulkCache.clearCache();
        gamepad1.readButtons();
        gamepad2.readButtons();

        // Drive first, with this loop's sticks and heading
        pinpointOdometrySubsystem.update();
        driveSubsystem.driveFieldCentric(
                scale(gamepad1.getLeftY()),
                scale(gamepad1.getLeftX()),
                scale(gamepad1.getRightX()),
                pinpointOdometrySubsystem.getPose().getHeading());
        driveSubsystem.update();

        // Then everything else
        commandScheduler.run();
        shooterSubsystem.update();
        intakeSubsystem.update();

        opMode.telemetry.addData("Loop (ms)", loopTimer.milliseconds());
        opMode.telemetry.addData("Heading (deg)", Math.toDegrees(pinpointOdometrySubsystem.getPose().getHeading()));
        opMode.telemetry.update();
        loopTimer.reset();
    }

    public void init(){
        commandScheduler.reset();

        commandScheduler.schedule(
                shooterSubsystem.shooterSetVelocityRunCommand(
                        () -> (pinpointOdometrySubsystem.getDistanceTo(goalPos)-22),
                        robotHardware.voltageSensor::getVoltage),
                new RunCommand(() -> {
                    // 0 = intaking, 1 = holding, 2 = shooting, 3 = ejecting
                    if (gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
                        intakeState = 0;
                    } else if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                        intakeState = 3;
                    } else if (gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
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

                    // Goal, from the driver's view
                    if (gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        goalPos = goalPosT1; // hive on right side
                    }
                    if (gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        goalPos = goalPosT2; // hive on left side
                    }

                    // Relocalize
                    if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
                        pinpointOdometrySubsystem.resetTo(resetPoseRA);
                    } else if (gamepad1.wasJustPressed(GamepadKeys.Button.B)) {
                        pinpointOdometrySubsystem.resetTo(resetPoseRS);
                    } else if (gamepad1.wasJustPressed(GamepadKeys.Button.X)) {
                        pinpointOdometrySubsystem.resetTo(resetPoseBA);
                    } else if (gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
                        pinpointOdometrySubsystem.resetTo(resetPoseBS);
                    }
                })
        );

        for (Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }
}
