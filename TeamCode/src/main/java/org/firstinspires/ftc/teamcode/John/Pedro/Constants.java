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
        c.yPodOffset.set(0.9375); // (1-1/16) = 0.9375
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Built on each call so the heading values in JohnTuning (live-editable from FTC Dashboard) take
     * effect for every new follower / algorithm.
     */
    public static ForesightConfig foresightConfig() {
        return foresightConfig(JohnTuning.HEADING_KP, JohnTuning.HEADING_KS, JohnTuning.HEADING_KS_DEADBAND_DEG,
                JohnTuning.HEADING_BRAKE_A, JohnTuning.HEADING_BRAKE_B);
    }

    /** Same, with explicit heading values (used by the heading calibration sweep). */
    public static ForesightConfig foresightConfig(double headingKp, double headingKs, double headingKsDeadbandDeg,
                                                  double headingBrakeA, double headingBrakeB) {
        return new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.41120344321980973);
                Controller secondaryTranslationalForward = Controller.proportional(0.1519286847113561);
                Controller primaryTranslationalLateral = Controller.proportional(0.676814632913156);
                Controller secondaryTranslationalLateral = Controller.proportional(0.25006492204134684);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010186903129938));
                c.brake.set(Controller.proportionalFeedforward(0.0086588676604473));

                c.headingFeedback.set(Controller.sum(
                        Controller.proportional(headingKp),
                        headingStaticFeedforward(headingKs, Math.toRadians(headingKsDeadbandDeg))));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(headingBrakeA, headingBrakeB));

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