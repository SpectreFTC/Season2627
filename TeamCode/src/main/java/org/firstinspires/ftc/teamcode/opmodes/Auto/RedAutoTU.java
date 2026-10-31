package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@Disabled
@Autonomous(group = "COMP", name = "RED_RUN_AUTO_UT")
public class RedAutoTU extends OpMode {
    CommandScheduler scheduler;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.9401, 132.5623, 90);
    private final Pose path1 = poseFactory.of(45.9433, 109.5729, -155.7878);
    private final Pose path1Control1 = poseFactory.of(61.9493, 116.7704, 0);
    private final Pose point2 = poseFactory.of(46.4757, 132.2671, 88.6561);
    private final Pose point3 = poseFactory.of(45.8855, 109.1401, 88.5384);
    private final Pose point4 = poseFactory.of(61.1338, 132.4276, -87.5934);
    private final Pose point4Control1 = poseFactory.of(61.7878, 116.8667, 0);
    private final Pose point5 = poseFactory.of(14.5803, 118.1297, -162.9267);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).tangent();
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
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
                        new InstantCommand(intakeSubsystem::flowerIntakeRampDown),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(700),
                        new InstantCommand(intakeSubsystem::intake),
                        new FollowPathCommand(follower, Paths.path(path1(), path2(), path3(), path4())),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(700),
                        new InstantCommand(intakeSubsystem::hold),
                        new FollowPathCommand(follower, path5())
                )

        );
    }

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
    }

}
