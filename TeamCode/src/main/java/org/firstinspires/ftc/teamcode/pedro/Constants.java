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
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("ei0");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.642850590503122);
        c.yPodOffset.set(-6.14031513844888);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3193013980930619);
                Controller secondaryTranslationalForward = Controller.proportional(0.11797333470489527);
                Controller primaryTranslationalLateral = Controller.proportional(0.46932391071059437);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17340264444170753);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013792903832013459));
                c.brake.set(Controller.proportionalFeedforward(0.01172396825721144));

                c.headingFeedback.set(Controller.proportional(5.0093373061821405));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05267241738999578, 0.0035603950048225265));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0899016525902602, 0.05886989147232648));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012286736079455676, 0.0014090710878274083));

                c.maxAchievableForwardVelocity.set(74.01916443405634);
                c.maxAchievableStrafeVelocity.set(59.05875220954137);
                c.naturalForwardDeceleration.set(32.8262588460505);
                c.naturalStrafeDeceleration.set(71.7821145646298);
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