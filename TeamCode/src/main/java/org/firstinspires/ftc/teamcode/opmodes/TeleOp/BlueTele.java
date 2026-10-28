package org.firstinspires.ftc.teamcode.opmodes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.PinpointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;
@TeleOp(group = "COMP", name = "BLUE_RUN_TELEOP")
public class BlueTele extends OpMode {
    MainTeleop tele;
    RobotHardware robotHardware;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    MecanumDriveSubsystem drive;
    PinpointOdometrySubsystem odo;
    CommandScheduler commandScheduler;

    @Override
    public void init() {
       robotHardware = RobotHardware.getInstance(this);
       shooter = new ShooterSubsystem(
               robotHardware.hoodServo,
               robotHardware.shooterMotor1,
               robotHardware.shooterMotor2);
       intake = new IntakeSubsystem(robotHardware);
       drive = new MecanumDriveSubsystem(robotHardware);
       odo = new PinpointOdometrySubsystem(robotHardware, new Pose2d());
       commandScheduler = CommandScheduler.getInstance();
       tele = new MainTeleop(gamepad1, gamepad2, this, shooter, intake, true);
       tele.init();
    }

    @Override
    public void loop() {
       tele.loop();
    }
}
