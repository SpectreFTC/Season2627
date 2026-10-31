package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.pedropathing.follower.Follower;
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

@Autonomous(group = "COMP", name = "RED_RUN_AUTO_1T")
public class RedAutoT1 extends OpMode {
    CommandScheduler scheduler;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.0721, 8.4639, 90);
    private final Pose path1 = poseFactory.of(55.0721, 23.7057, 90);
    private final Pose point2 = poseFactory.of(11.7041, 102.877, 90.7449);
    private final Pose point2Control1 = poseFactory.of(12.0607, 59.0041, 0);

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
                        new FollowPathCommand(follower, Paths.path(path1(), path2())),
                        new InstantCommand(intakeSubsystem::flowerIntakeRampDown)
                )

        );
    }

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
    }

}
