package org.firstinspires.ftc.teamcode.John.Pedro;

import com.acmerobotics.dashboard.config.Config;

/**
 * John's Pedro controller values (heading, forward, strafe), live-editable from FTC Dashboard.
 * Constants builds the Foresight controller from these each time it is asked (create() /
 * foresightConfig(), via JohnGains.current()), and the [John] Hold Test swaps in a new controller
 * as soon as a value changes.
 *
 * Something that oscillates while holding usually means its P is too high; one that stops short
 * of the target needs its KS (friction push).
 */
@Config
public class JohnTuning {

    /** Proportional heading feedback. */
    public static double HEADING_KP = 5.2; // 8.213497192073243;
    /**
     * Static friction feedforward added to the heading output: KS * sign(error) while the error is
     * larger than HEADING_KS_DEADBAND_DEG. Lets the robot finish a turn instead of stopping short.
     */
    public static double HEADING_KS = 0.0;
    public static double HEADING_KS_DEADBAND_DEG = 0.25;
    /** Heading brake coefficients (Vector2D.cartesian(A, B) in Pedro's ForesightConfig). */
    public static double HEADING_BRAKE_A = 0.05; // tuner: 0.09861967506750287
    public static double HEADING_BRAKE_B = 0.006; // tuner: 0.004613531083625517

    /**
     * Forward / strafe position controllers are piecewise: the SMALL P below
     * TRANSLATIONAL_THRESHOLD_IN of error, the LARGE P above it (tuner values).
     */
    public static double FORWARD_P_SMALL = 0.1519286847113561;
    public static double FORWARD_P_LARGE = 0.41120344321980973;
    public static double STRAFE_P_SMALL = 0.25006492204134684;
    public static double STRAFE_P_LARGE = 0.676814632913156;
    public static double TRANSLATIONAL_THRESHOLD_IN = 2.5;
    /**
     * Static friction push added to the forward / strafe output: KS * sign(error) while that error
     * is larger than TRANSLATIONAL_KS_DEADBAND_IN. Lets the robot finish a move instead of stopping
     * short (it stops about 0.4" short forward and 0.6" short sideways without it).
     */
    public static double FORWARD_KS = 0.06;
    public static double STRAFE_KS = 0.15;
    public static double TRANSLATIONAL_KS_DEADBAND_IN = 0.1;

    private JohnTuning() {}
}
