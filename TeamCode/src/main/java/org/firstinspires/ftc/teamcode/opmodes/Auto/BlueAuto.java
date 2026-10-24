package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(group = "COMP", name = "BLUE_RUN_AUTO")
public class BlueAuto extends OpMode {
    CommandScheduler scheduler;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

//    private final Pose start = poseFactory.of(86.1632, 132.81, -90);
    private final Pose start = poseFactory.of(86.1632, 132.81, -90);
    private final Pose path1 = poseFactory.of(86.3877, 116.9025, -89.1913);
    private final Pose point2 = poseFactory.of(130.9279, 43.9614, -90.2473);
    private final Pose point2Control1 = poseFactory.of(131.4141, 104.1128, 0);

    public Path path1() {
        return Paths.line(start, path1).tangent();
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).tangent();
    }

    @Override
    public void init() {
        scheduler = CommandScheduler.getInstance();
        scheduler.reset();
        RobotHardware r = RobotHardware.getInstance(this);
        shooterSubsystem = new ShooterSubsystem(r.hoodServo, r.shooterMotor1, r.shooterMotor2);
        intakeSubsystem = new IntakeSubsystem(r);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        scheduler.schedule(
                new RunCommand(() -> {
                    shooterSubsystem.runAtVelocity(RobotConstants.setVelocity, r.voltageSensor::getVoltage);
                    shooterSubsystem.setHoodAngle(RobotConstants.setHoodAngle);
                }),
                new SequentialCommandGroup(
                    new InstantCommand(intakeSubsystem::hold),
                    new InstantCommand(intakeSubsystem::flowerIntakeRampUp),
                    new WaitCommand(5000),
                    new InstantCommand(intakeSubsystem::shoot),
                    new WaitCommand(2000),
                    new InstantCommand(intakeSubsystem::hold),
                    new FollowPathCommand(follower, Paths.path(path1(), path2()))
                )

        );
    }

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
    }


}
