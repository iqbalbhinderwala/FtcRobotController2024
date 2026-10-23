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
     * Built on each call so the values in JohnTuning (live-editable from FTC Dashboard) take effect
     * for every new follower / algorithm.
     */
    public static ForesightConfig foresightConfig() {
        return foresightConfig(JohnGains.current());
    }

    /** Same, with explicit values (used by the calibration sweeps). */
    public static ForesightConfig foresightConfig(JohnGains g) {
        return new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(g.forwardPLarge);
                Controller secondaryTranslationalForward = Controller.proportional(g.forwardPSmall);
                Controller primaryTranslationalLateral = Controller.proportional(g.strafePLarge);
                Controller secondaryTranslationalLateral = Controller.proportional(g.strafePSmall);

                c.forwardTranslational.set(Controller.sum(
                        Controller.piecewise(secondaryTranslationalForward).put(g.translationalThresholdIn, primaryTranslationalForward),
                        staticFeedforward(g.forwardKs, g.translationalKsDeadbandIn)));
                c.strafeTranslational.set(Controller.sum(
                        Controller.piecewise(secondaryTranslationalLateral).put(g.translationalThresholdIn, primaryTranslationalLateral),
                        staticFeedforward(g.strafeKs, g.translationalKsDeadbandIn)));

                c.coast.set(Controller.proportionalFeedforward(0.010186903129938));
                c.brake.set(Controller.proportionalFeedforward(0.0086588676604473));

                c.headingFeedback.set(Controller.sum(
                        Controller.proportional(g.headingKp),
                        staticFeedforward(g.headingKs, Math.toRadians(g.headingKsDeadbandDeg))));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(g.headingBrakeA, g.headingBrakeB));

                c.linearBrakeCoefficients.set(Matrix.diag(0.15218656729826474, 0.11945174018596512));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014714148194534546, 0.0012325337662412374));

                c.maxAchievableForwardVelocity.set(88.62593753486438);
                c.maxAchievableStrafeVelocity.set(69.59926334249484);
                c.naturalForwardDeceleration.set(30.482755979464184);
                c.naturalStrafeDeceleration.set(68.61843691164408);
            }
        );
    }

    /**
     * Static friction feedforward: ks * sign(error), but 0 within deadband of the target so it does
     * not chatter around zero error (Pedro's Controller.staticFeedforward has no deadband). The
     * deadband is in the controller's error units (radians for heading, inches for translation).
     */
    private static Controller staticFeedforward(double ks, double deadband) {
        return (target, error) -> Math.abs(error) > deadband ? ks * Math.signum(error) : 0.0;
    }

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig())
        );
    }
}