package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.PinpointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.globals.BulkCache;
import org.firstinspires.ftc.teamcode.globals.RobotHardware;

@Config
@TeleOp
public class PinpointTest extends OpMode {
    RobotHardware r;
    FtcDashboard dashboard;
    MultipleTelemetry multipleTelemetry;
    PinpointOdometrySubsystem odo;
    BulkCache bulkCache;
    public static double x;
    public static double y;
    public static double ___resetX;
    public static double __resetY;
    public static double _resetH;
    @Override
    public void init() {
        r = RobotHardware.getInstance(this);
        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        odo = new PinpointOdometrySubsystem(r, new Pose2d());
        odo.init();
        bulkCache = new BulkCache(hardwareMap);
    }

    @Override
    public void loop() {
        bulkCache.clearCache();
        odo.update();
        if(gamepad1.a){
            odo.resetTo(new Pose2d(___resetX, __resetY, Rotation2d.fromDegrees(_resetH)));
        }
        multipleTelemetry.addLine(odo.toString());

        multipleTelemetry.addData("distance to: X: "+x+" Y: "+y , odo.getDistanceTo(new Translation2d(x, y)));

        multipleTelemetry.update();

    }
}
