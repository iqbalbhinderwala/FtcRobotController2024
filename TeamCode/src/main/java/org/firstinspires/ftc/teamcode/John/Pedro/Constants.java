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

import java.util.OptionalDouble;

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

    /** goBILDA 4-bar pod nominal 19.894 ticks/mm, times 1.00995 from the push test. */
    public static final double ENCODER_TICKS_PER_MM = 19.894 * 1.00995;

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Measured, not the 4-bar pod's nominal 19.894 ticks/mm: hand pushes of 96.0" (tape measure)
        // read 96.88" and 97.03" (r0243, 2026-11-01), so the pods read about 1.0% long.
        c.encoderResolutionUnit.set(DistanceUnit.MM);
        c.ticksPerUnit.set(OptionalDouble.of(ENCODER_TICKS_PER_MM));
        c.xPodOffset.set(0.0);
        // The strafe pod is 15/16" toward the intake B end (+X, forward = positive). It was -0.9375
        // (sign left over from the front being reversed on 2026-10-22): [John] Odometry Test spins
        // r0234 / r0235 showed a 2.3-2.6" forward lever arm, which swung the pose up to ~5" on big turns.
        c.yPodOffset.set(0.9375); // 1-1/16 = -0.9375
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
                        staticFeedforward(g.forwardKs, g.translationalKsDeadbandIn, g.translationalKsRampIn)));
                c.strafeTranslational.set(Controller.sum(
                        Controller.piecewise(secondaryTranslationalLateral).put(g.translationalThresholdIn, primaryTranslationalLateral),
                        staticFeedforward(g.strafeKs, g.translationalKsDeadbandIn, g.translationalKsRampIn)));

                // John v2 (2026-11-13): the average of two Foresight tuner runs (v1 values in the
                // comments). The strafe braking values are provisional: the lowest-speed strafe brake
                // stage stalled and was pushed by hand. Heading KP / brake stay in JohnTuning (our
                // Heading Calibration; the tuner gave KP 14.9, brake 0.116 / 0.0063).
                c.coast.set(Controller.proportionalFeedforward(0.012946));   // v1 0.010187
                c.brake.set(Controller.proportionalFeedforward(0.011004));   // v1 0.008659

                c.headingFeedback.set(Controller.sum(
                        Controller.proportional(g.headingKp),
                        staticFeedforward(g.headingKs, Math.toRadians(g.headingKsDeadbandDeg), 0)));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(g.headingBrakeA, g.headingBrakeB));

                // Strafe: [John] Brake Calibration r1050 + r1051 (12 trials, fresh battery; the tuner
                // gave 0.087859 / 0.0023155 with a stalled trial). Forward: tuner, to be measured.
                c.linearBrakeCoefficients.set(Matrix.diag(0.179222, 0.081280));         // v1 0.152187, 0.119452
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0022750, 0.0028440));    // v1 0.0014714, 0.0012325

                c.maxAchievableForwardVelocity.set(73.234);   // v1 88.63
                c.maxAchievableStrafeVelocity.set(51.489);    // v1 69.60
                c.naturalForwardDeceleration.set(27.300);     // v1 30.48
                c.naturalStrafeDeceleration.set(75.029);      // v1 68.62
            }
        );
    }

    /**
     * Static friction feedforward: ks * sign(error), but 0 within deadband of the target so it does
     * not chatter around zero error (Pedro's Controller.staticFeedforward has no deadband). The
     * deadband is in the controller's error units (radians for heading, inches for translation).
     * With ramp > 0 the push grows linearly from 0 at the deadband to the full ks at deadband + ramp
     * (no on/off step for a small wobble to flip: Pedro feeds the predicted-stop error, which moves
     * with every bit of speed noise). ramp 0 = the plain step.
     */
    private static Controller staticFeedforward(double ks, double deadband, double ramp) {
        return (target, error) -> {
            double beyond = Math.abs(error) - deadband;
            if (beyond <= 0) return 0.0;
            double scale = ramp > 0 ? Math.min(1.0, beyond / ramp) : 1.0;
            return ks * scale * Math.signum(error);
        };
    }

    public static Follower create(HardwareMap h) {
        return create(h, foresightConfig());
    }

    /**
     * With a given Foresight config, so the caller can keep it and change it per path, e.g.
     * path.with(config.maxVelocityConstraint.at(12.0)) to drive one path slower.
     */
    public static Follower create(HardwareMap h, ForesightConfig config) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(config)
        );
    }
}