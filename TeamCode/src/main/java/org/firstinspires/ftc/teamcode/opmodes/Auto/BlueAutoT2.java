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
@Autonomous(group = "COMP", name = "BLUE_RUN_AUTO_2T")
public class BlueAutoT2 extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86.0755, 132.8316, -90);
    private final Pose path1 = poseFactory.of(86.0508, 125.6834, -90);
    private final Pose point2 = poseFactory.of(133.3528, 133.5589, 0.5504);
    private final Pose point2Control1 = poseFactory.of(103.5093, 133.2722, 0);
    private final Pose point3 = poseFactory.of(83.1011, 24.0517, 90);
    private final Pose point3Control1 = poseFactory.of(93.557, 110.3319, 0);
    private final Pose point3Control2 = poseFactory.of(124.9631, 59.1512, 0);
    private final Pose point3Control3 = poseFactory.of(109.8546, 22.6095, 0);
    private final Pose point3Segment1Heading = poseFactory.of(83.1011, 24.0517, 0);
    private final Pose point3Segment2Start = poseFactory.of(83.1011, 24.0517, 0);
    private final Pose point3Segment2End = poseFactory.of(83.1011, 24.0517, 90);
    private final Pose point4 = poseFactory.of(94.2942, 12.0267, -88.0823);
    private final Pose point4Control1 = poseFactory.of(93.7947, 26.9449, 0);
    private final Pose point5 = poseFactory.of(82.655, 30.3275, -19.3156);
    private final Pose point5Control1 = poseFactory.of(94.3774, 26.2188, 0);
    private final Pose point6 = poseFactory.of(82.66, 24.539, 90.0491);
    private final Pose point7 = poseFactory.of(130.4242, 94.5843, 0);
    private final Pose point7Control1 = poseFactory.of(130.4041, 28.1003, 0);
    private final Pose point7Control2 = poseFactory.of(96.4127, 95.3113, 0);
    private final Pose point7Segment2Start = poseFactory.of(130.4242, 94.5843, 84.3661);
    private final Pose point7Segment2End = poseFactory.of(130.4242, 94.5843, 0);
    private final Pose point8 = poseFactory.of(82.2192, 24.8474, 90);
    private final Pose point8Control1 = poseFactory.of(98.8238, 93.2016, 0);
    private final Pose point8Control2 = poseFactory.of(128.4518, 34.3327, 0);
    private final Pose point8Segment2Start = poseFactory.of(82.2192, 24.8474, 75.477);
    private final Pose point8Segment2End = poseFactory.of(82.2192, 24.8474, 90);
    private final Pose point9 = poseFactory.of(130.9043, 35.058, -0.5426);
    private final Pose point9Control1 = poseFactory.of(88.4267, 35.4603, 0);

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
        return Paths.curve(point3, point4Control1, point4).tangent();
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).heading(Interpolator.piecewise().until(1, Interpolator.tangent.reverse()));
    }

    public Path path6() {
        return Paths.line(point5, point6).reverseTangent();
    }

    public Path path7() {
        return Paths.curve(point6, point7Control1, point7Control2, point7).heading(Interpolator.piecewise().until(0.6293, Interpolator.tangent).until(1, Interpolator.linear(point7Segment2Start, point7Segment2End)));
    }

    public Path path8() {
        return Paths.curve(point7, point8Control1, point8Control2, point8).heading(Interpolator.piecewise().until(0.5621, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point8Segment2Start, point8Segment2End)));
    }

    public Path path9() {
        return Paths.curve(point8, point9Control1, point9).tangent();
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
                    shooterSubsystem.runAtVelocity(RobotConstants.autonSetVelocityBlue, r.voltageSensor::getVoltage);
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
                        new WaitCommand(550),
                        new FollowPathCommand(follower, path5()),
                        new FollowPathCommand(follower, path6()),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(1000),
                        new InstantCommand(intakeSubsystem::intake),
                        new FollowPathCommand(follower, path7()),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, path8()),
                        new WaitCommand(500),
                        new InstantCommand(intakeSubsystem::shoot),
                        new WaitCommand(1000),
                        new InstantCommand(intakeSubsystem::flowerIntakeRampUp),
                        new FollowPathCommand(follower, path9()),
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
