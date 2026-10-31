package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("ei0");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.8592802032711);
        c.yPodOffset.set(-7.145110002652866);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("cm1");
        c.frontRightName.set("cm3");
        c.backLeftName.set("cm0");
        c.backRightName.set("cm2");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27685919684517607);
                Controller secondaryTranslationalForward = Controller.proportional(0.10229207542030537);
                Controller primaryTranslationalLateral = Controller.proportional(0.44340072121684926);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1638247186041493);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01205662189231498));
                c.brake.set(Controller.proportionalFeedforward(0.010248128608467733));

                c.headingFeedback.set(Controller.proportional(4.391285764598986));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05707876494231827, 0.004397667160103959));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08751129036500438, 0.06639366071278692));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001488277784567253, 0.0014117741872194793));

                c.maxAchievableForwardVelocity.set(84.84895853355498);
                c.maxAchievableStrafeVelocity.set(65.71148143547994);
                c.naturalForwardDeceleration.set(34.056056319234905);
                c.naturalStrafeDeceleration.set(70.14436831359905);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}