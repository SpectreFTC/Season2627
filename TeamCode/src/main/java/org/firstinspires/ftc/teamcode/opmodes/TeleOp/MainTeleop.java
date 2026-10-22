package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
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
    Pose2d resetPose, resetPose1, resetPose2, resetPose3;
    Translation2d goalPos, goalPosT1, goalPosT2, goalPosB1, goalPosB2, goalPosR1, goalPosR2;
    int intakeState = 0;
    boolean flowerMechDown = false;
    //the robot should always be facing towards you when relocalizing in any corner
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
    public static double scale(float v){
        return Math.signum(v)*Math.sqrt(Math.abs(v));
    }
    public void loop(){
       commandScheduler.run();
    }

    public void init(){
        commandScheduler.reset();

        commandScheduler.schedule(
            shooterSubsystem.shooterSetVelocityRunCommand(()-> pinpointOdometrySubsystem.getDistanceTo(goalPos), robotHardware.voltageSensor::getVoltage),
            new RunCommand(()->{
                //intake state 0 is intake-ing, intake state 1 is holding,
                // intake state 2 is shooting, intake state 3 is ejecting
                if(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0 && gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                    intakeState = 0;
                }else if(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0){
                   intakeState = 0;
                }else if(gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                    intakeState = 3;
                }else if(gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
                    intakeState = 2;
                }else{
                    intakeState = 1;
                }
                if(gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                   flowerMechDown = !flowerMechDown;
                }
                //drivers view
                if(gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    goalPos = goalPosT1; // hive on right side
                }
                if(gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    goalPos = goalPosT2; // hive on left side
                }

            }),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(intakeSubsystem::intake), new InstantCommand(), ()->intakeState==0)
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(intakeSubsystem::hold), new InstantCommand(), ()->intakeState==1)
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(intakeSubsystem::shoot), new InstantCommand(), ()->intakeState==2)
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(intakeSubsystem::eject), new InstantCommand(), ()->intakeState==3)
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(()->pinpointOdometrySubsystem.resetTo(resetPose)), new InstantCommand(), ()-> gamepad1.wasJustPressed(GamepadKeys.Button.A))
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(()->pinpointOdometrySubsystem.resetTo(resetPose1)), new InstantCommand(), ()-> gamepad1.wasJustPressed(GamepadKeys.Button.B))
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(()->pinpointOdometrySubsystem.resetTo(resetPose2)), new InstantCommand(), ()-> gamepad1.wasJustPressed(GamepadKeys.Button.X))
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(()->pinpointOdometrySubsystem.resetTo(resetPose3)), new InstantCommand(), ()-> gamepad1.wasJustPressed(GamepadKeys.Button.Y))
            ),
            new RepeatCommand(
                    new ConditionalCommand(new InstantCommand(intakeSubsystem::intakeFromFlower), new InstantCommand(intakeSubsystem::flowerIntakeRampUp), ()-> flowerMechDown)
            )
        );
        for (Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void update(){
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
    }
}
