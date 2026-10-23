package org.firstinspires.ftc.teamcode.John.Pedro;

import com.acmerobotics.dashboard.config.Config;

/**
 * John's Pedro heading-controller values, live-editable from FTC Dashboard. Constants builds the
 * Foresight controller from these each time it is asked (create() / foresightConfig()), and the
 * [John] Hold Test swaps in a new controller as soon as a value changes.
 *
 * Defaults are the values from the Pedro tuning run. A heading that oscillates while holding
 * usually means HEADING_KP is too high; one that stops short of the target needs HEADING_KS.
 */
@Config
public class JohnTuning {

    /** Proportional heading feedback. */
    public static double HEADING_KP = 8.213497192073243;
    /**
     * Static friction feedforward added to the heading output: KS * sign(error) while the error is
     * larger than HEADING_KS_DEADBAND_DEG. Lets the robot finish a turn instead of stopping short.
     */
    public static double HEADING_KS = 0.0;
    public static double HEADING_KS_DEADBAND_DEG = 0.25;
    /** Heading brake coefficients (Vector2D.cartesian(A, B) in Pedro's ForesightConfig). */
    public static double HEADING_BRAKE_A = 0.09861967506750287;
    public static double HEADING_BRAKE_B = 0.004613531083625517;

    private JohnTuning() {}
}
