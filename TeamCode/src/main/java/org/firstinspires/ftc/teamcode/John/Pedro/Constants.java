package org.firstinspires.ftc.teamcode.John.Pedro;

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
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        // Optional
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(-0.9375); // -(1-1/16) = -0.9375
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Built on each call so the heading values in JohnTuning (live-editable from FTC Dashboard) take
     * effect for every new follower / algorithm.
     */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4004033121595838);
                Controller secondaryTranslationalForward = Controller.proportional(0.1479383248694195);
                Controller primaryTranslationalLateral = Controller.proportional(0.6245542308448);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23075609990075086);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010204641654105027));
                c.brake.set(Controller.proportionalFeedforward(0.008673945405989272));

                c.headingFeedback.set(Controller.sum(
                        Controller.proportional(JohnTuning.HEADING_KP),
                        headingStaticFeedforward(JohnTuning.HEADING_KS, Math.toRadians(JohnTuning.HEADING_KS_DEADBAND_DEG))));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(JohnTuning.HEADING_BRAKE_A, JohnTuning.HEADING_BRAKE_B));

                c.linearBrakeCoefficients.set(Matrix.diag(0.17411776699227124, 0.12495511055294069));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.680280106685293E-4, 0.0012149559218922641));

                c.maxAchievableForwardVelocity.set(85.44499251313609);
                c.maxAchievableStrafeVelocity.set(70.40932439236533);
                c.naturalForwardDeceleration.set(27.225903029015566);
                c.naturalStrafeDeceleration.set(67.8340877861556);
            }
        );
    }

    /**
     * Static friction feedforward: ks * sign(error), but 0 within deadbandRad of the target so it
     * does not chatter around zero error (Pedro's Controller.staticFeedforward has no deadband).
     */
    private static Controller headingStaticFeedforward(double ks, double deadbandRad) {
        return (target, error) -> Math.abs(error) > deadbandRad ? ks * Math.signum(error) : 0.0;
    }

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig())
        );
    }
}