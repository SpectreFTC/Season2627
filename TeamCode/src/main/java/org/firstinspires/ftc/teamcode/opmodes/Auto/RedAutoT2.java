package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(group = "COMP", name = "RED_RUN_AUTO_2T")
public class RedAutoT2 extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.0941, 8, 90);
    private final Pose path1 = poseFactory.of(54.8881, 12.0848, 90);
    private final Pose point2 = poseFactory.of(7.4341, 8.0025, 179.8318);
    private final Pose point2Control1 = poseFactory.of(43.3583, 7.897, 0);
    private final Pose point3 = poseFactory.of(55.4473, 124.6176, 270);
    private final Pose point3Control1 = poseFactory.of(33.9495, 25.5406, 0);
    private final Pose point3Control2 = poseFactory.of(19.6987, 73.2831, 0);
    private final Pose point3Control3 = poseFactory.of(25.6069, 117.3656, 0);
    private final Pose point3Segment1Heading = poseFactory.of(55.4473, 124.6176, 180);
    private final Pose point3Segment2Start = poseFactory.of(55.4473, 124.6176, 180);
    private final Pose point3Segment2End = poseFactory.of(55.4473, 124.6176, -90);
    private final Pose point4 = poseFactory.of(46.4753, 118.4235, -151.44);
    private final Pose point4Control1 = poseFactory.of(51.057, 120.9174, 0);
    private final Pose point5 = poseFactory.of(46.3806, 126.4634, 90.6749);
    private final Pose point6 = poseFactory.of(55.4451, 116.3719, 170.4936);
    private final Pose point6Control1 = poseFactory.of(48.0516, 117.61, 0);
    private final Pose point7 = poseFactory.of(55.2917, 124.2539, -88.8847);
    private final Pose point8 = poseFactory.of(16.2821, 46.9345, -180);
    private final Pose point8Control1 = poseFactory.of(9.9208, 123.7623, 0);
    private final Pose point8Control2 = poseFactory.of(46.2264, 48.3862, 0);
    private final Pose point8Segment2Start = poseFactory.of(16.2821, 46.9345, -91.0937);
    private final Pose point8Segment2End = poseFactory.of(16.2821, 46.9345, 180);
    private final Pose point9 = poseFactory.of(55.3474, 124.7519, -90);
    private final Pose point9Control1 = poseFactory.of(43.3833, 43.7401, 0);
    private final Pose point9Control2 = poseFactory.of(11.4109, 125.2841, 0);
    private final Pose point9Segment2Start = poseFactory.of(55.3474, 124.7519, -94.6298);
    private final Pose point9Segment2End = poseFactory.of(55.3474, 124.7519, -90);
    private final Pose point10 = poseFactory.of(10.9645, 99.9198, 1.1144);
    private final Pose point10Control1 = poseFactory.of(40.958, 100.5032, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).tangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3Control3, point3).heading(Interpolator.piecewise().until(0.1018, Interpolator.constant(point3Segment1Heading)).until(1, Interpolator.linear(point3Segment2Start, point3Segment2End)));
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).heading(Interpolator.piecewise().until(1, Interpolator.tangent));
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6).reverseTangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.curve(point7, point8Control1, point8Control2, point8).heading(Interpolator.piecewise().until(0.6558, Interpolator.tangent).until(1, Interpolator.linear(point8Segment2Start, point8Segment2End)));
    }

    public Path path9() {
        return Paths.curve(point8, point9Control1, point9Control2, point9).heading(Interpolator.piecewise().until(0.5234, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point9Segment2Start, point9Segment2End)));
    }

    public Path path10() {
        return Paths.curve(point9, point10Control1, point10).reverseTangent();
    }
    CommandScheduler scheduler;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    Follower follower;

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
                    shooterSubsystem.runAtVelocity(RobotConstants.autonSetVelocityRed, r.voltageSensor::getVoltage);
                    shooterSubsystem.setHoodAngle(RobotConstants.setHoodAngle);
                }),
                new SequentialCommandGroup(
                        new InstantCommand(intakeSubsystem::hold),
                        new InstantCommand(intakeSubsystem::flowerIntakeRampUp),
                        new FollowPathCommand(follower, path1()),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(1000),
                        new InstantCommand(intakeSubsystem::intake),
                        new FollowPathCommand(follower, path2(), 0.5),
                        new WaitCommand(400),
                        new InstantCommand(intakeSubsystem::hold),
                        new FollowPathCommand(follower, path3(), 1),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(1000),
                        new InstantCommand(intakeSubsystem::flowerIntakeRampDown),
                        new InstantCommand(intakeSubsystem::intake),
                        new FollowPathCommand(follower, path4()),
                        new FollowPathCommand(follower, path5()),
                        new WaitCommand(550),
                        new FollowPathCommand(follower, path6()),
                        new FollowPathCommand(follower, path7()),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(1000),
                        new InstantCommand(intakeSubsystem::intake),
                        new FollowPathCommand(follower, path8()),
                        new WaitCommand(500),
                        new FollowPathCommand(follower, path9()),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(1000),
                        new InstantCommand(intakeSubsystem::flowerIntakeRampUp),
                        new FollowPathCommand(follower, path10()),
                        new InstantCommand(intakeSubsystem::hold)
                )
        );
    }

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
    }

}
