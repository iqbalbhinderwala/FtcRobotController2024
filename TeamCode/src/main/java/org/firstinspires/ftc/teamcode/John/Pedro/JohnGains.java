package org.firstinspires.ftc.teamcode.John.Pedro;

import java.util.Locale;

/**
 * One set of John's tunable controller values (see JohnTuning for what each one does).
 * Constants.foresightConfig(gains) builds the Foresight controller from it; the calibration
 * OpModes change a copy instead of the JohnTuning globals.
 */
public class JohnGains {

    public double headingKp, headingKs, headingKsDeadbandDeg, headingBrakeA, headingBrakeB;
    public double forwardPSmall, forwardPLarge, strafePSmall, strafePLarge, translationalThresholdIn;
    public double forwardKs, strafeKs, translationalKsDeadbandIn;

    /** The values currently in JohnTuning (live from the Dashboard). */
    public static JohnGains current() {
        JohnGains g = new JohnGains();
        g.headingKp = JohnTuning.HEADING_KP;
        g.headingKs = JohnTuning.HEADING_KS;
        g.headingKsDeadbandDeg = JohnTuning.HEADING_KS_DEADBAND_DEG;
        g.headingBrakeA = JohnTuning.HEADING_BRAKE_A;
        g.headingBrakeB = JohnTuning.HEADING_BRAKE_B;
        g.forwardPSmall = JohnTuning.FORWARD_P_SMALL;
        g.forwardPLarge = JohnTuning.FORWARD_P_LARGE;
        g.strafePSmall = JohnTuning.STRAFE_P_SMALL;
        g.strafePLarge = JohnTuning.STRAFE_P_LARGE;
        g.translationalThresholdIn = JohnTuning.TRANSLATIONAL_THRESHOLD_IN;
        g.forwardKs = JohnTuning.FORWARD_KS;
        g.strafeKs = JohnTuning.STRAFE_KS;
        g.translationalKsDeadbandIn = JohnTuning.TRANSLATIONAL_KS_DEADBAND_IN;
        return g;
    }

    /** Writes these values to JohnTuning (until the app restarts). */
    public void applyToTuning() {
        JohnTuning.HEADING_KP = headingKp;
        JohnTuning.HEADING_KS = headingKs;
        JohnTuning.HEADING_KS_DEADBAND_DEG = headingKsDeadbandDeg;
        JohnTuning.HEADING_BRAKE_A = headingBrakeA;
        JohnTuning.HEADING_BRAKE_B = headingBrakeB;
        JohnTuning.FORWARD_P_SMALL = forwardPSmall;
        JohnTuning.FORWARD_P_LARGE = forwardPLarge;
        JohnTuning.STRAFE_P_SMALL = strafePSmall;
        JohnTuning.STRAFE_P_LARGE = strafePLarge;
        JohnTuning.TRANSLATIONAL_THRESHOLD_IN = translationalThresholdIn;
        JohnTuning.FORWARD_KS = forwardKs;
        JohnTuning.STRAFE_KS = strafeKs;
        JohnTuning.TRANSLATIONAL_KS_DEADBAND_IN = translationalKsDeadbandIn;
    }

    public JohnGains copy() {
        JohnGains g = new JohnGains();
        g.headingKp = headingKp;
        g.headingKs = headingKs;
        g.headingKsDeadbandDeg = headingKsDeadbandDeg;
        g.headingBrakeA = headingBrakeA;
        g.headingBrakeB = headingBrakeB;
        g.forwardPSmall = forwardPSmall;
        g.forwardPLarge = forwardPLarge;
        g.strafePSmall = strafePSmall;
        g.strafePLarge = strafePLarge;
        g.translationalThresholdIn = translationalThresholdIn;
        g.forwardKs = forwardKs;
        g.strafeKs = strafeKs;
        g.translationalKsDeadbandIn = translationalKsDeadbandIn;
        return g;
    }

    /** All values, for spotting a change. */
    public double[] toArray() {
        return new double[]{headingKp, headingKs, headingKsDeadbandDeg, headingBrakeA, headingBrakeB,
                forwardPSmall, forwardPLarge, strafePSmall, strafePLarge, translationalThresholdIn,
                forwardKs, strafeKs, translationalKsDeadbandIn};
    }

    public String summary() {
        return String.format(Locale.US, "heading KP %.3f KS %.3f/%.2fdeg brake %.4f/%.5f; "
                        + "forward P %.4f/%.4f KS %.3f; strafe P %.4f/%.4f KS %.3f; threshold %.2fin, KS deadband %.2fin",
                headingKp, headingKs, headingKsDeadbandDeg, headingBrakeA, headingBrakeB,
                forwardPSmall, forwardPLarge, forwardKs, strafePSmall, strafePLarge, strafeKs,
                translationalThresholdIn, translationalKsDeadbandIn);
    }
}
