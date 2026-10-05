package org.firstinspires.ftc.teamcode.Barry.Pedro;

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
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");

        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(1.8503937); // 4.7cm / 2.54
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig( c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.1252603487020835);
        Controller secondaryTranslationalForward = Controller.proportional(0.04628035182725961);
        Controller primaryTranslationalLateral = Controller.proportional(0.17083776075761348);
        Controller secondaryTranslationalLateral = Controller.proportional(0.06311998773089834);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.01612750293178795));
        c.brake.set(Controller.proportionalFeedforward(0.013708377492019757));

        c.headingFeedback.set(Controller.proportional(1.948321068754171));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056197523907207246, 0.002301180378543053));

        c.linearBrakeCoefficients.set(Matrix.diag(0.05725257704454846, 0.054336119916007425));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012013117630569616, 0.0012271732486635132));

        c.maxAchievableForwardVelocity.set(63.26491128886054);
        c.maxAchievableStrafeVelocity.set(54.187825071642045);
        c.naturalForwardDeceleration.set(54.2310494025295);
        c.naturalStrafeDeceleration.set(73.5520054245216);
    });

    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
//         return null;
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}