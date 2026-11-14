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
     * John v2 (2026-11-13): the average of two Foresight tuner runs (v1: forward 0.152 / 0.411, strafe
     * 0.250 / 0.677), except the strafe P: the tuner gave 1.1894 / 3.2191 (the lowest-speed strafe
     * stage stalled and was pushed by hand), which buzzed while holding after strafes; [John] Hold
     * Test r1037 settled on 0.4 / 1.5. The LARGE P (beyond 2.5" off) did not change the hold and keeps
     * John on a path: back to the tuner's 3.2191 after Auto r1063 drifted 10" off a path at 70 in/s
     * with 1.5 (the path watcher took it for a push).
     */
    public static double FORWARD_P_SMALL = 0.2387;
    public static double FORWARD_P_LARGE = 0.6462;
    public static double STRAFE_P_SMALL = 0.4;
    public static double STRAFE_P_LARGE = 3.2191;
    public static double TRANSLATIONAL_THRESHOLD_IN = 2.5;
    /**
     * Static friction push added to the forward / strafe output: KS * sign(error) while that error
     * is larger than TRANSLATIONAL_KS_DEADBAND_IN. Lets the robot finish a move instead of stopping
     * short (it stops about 0.4" short forward and 0.6" short sideways without it).
     * John v2 (Hold Test r1037): with the 0.1" deadband the forward push flipped sign every loop
     * around the target after a strafe (an 11-14 Hz buzz); 0.25" and forward KS 0.03 hold quietly,
     * stopping about 0.2-0.3" short.
     * r1039: a strafe KS of 0.15 or more still shook while holding (Pedro holds the predicted stop,
     * position + 0.179 x speed, so a small shudder crossed the deadband and flipped the push);
     * 0.10 holds quietly, about 0.3" short sideways.
     */
    public static double FORWARD_KS = 0.03;
    public static double STRAFE_KS = 0.10;
    public static double TRANSLATIONAL_KS_DEADBAND_IN = 0.25;
    /**
     * The friction push grows from 0 at the deadband to the full KS over this many more inches,
     * instead of switching fully on at the deadband (0 = the old on/off step). A small wobble in
     * the predicted stop then gives a small push, not a full flip.
     */
    public static double TRANSLATIONAL_KS_RAMP_IN = 0.25;

    private JohnTuning() {}
}
