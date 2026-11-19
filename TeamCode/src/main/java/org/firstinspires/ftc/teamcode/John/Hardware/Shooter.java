package org.firstinspires.ftc.teamcode.John.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * John's shooter: the flywheel (Flywheel) and the gate / indexer (Indexer), used the same way by
 * the autonomous and the TeleOp. Call update() every loop.
 *
 *  - Speed from the distance to the CELL (setDistance): RPM_DISTANCES_IN / RPM_VALUES, linear in
 *    between, clamped at the ends (robot center to the CELL's tag cluster center, from above; see
 *    ShootingZone). setRpmOverride() replaces the table (Shot Calibration).
 *  - Each ball fires only with the flywheel within the fire tolerance of its target: FIRE_TOLERANCE_NEAR_RPM
 *    up to TOLERANCE_NEAR_IN, FIRE_TOLERANCE_FAR_RPM from TOLERANCE_FAR_IN, linear in between. Far
 *    away the speed window that scores is narrow (2026-11-01, r0273: about 50 RPM at 62"), and with
 *    +/-100 the balls of one volley left up to 129 RPM apart.
 *  - setSpin(true) keeps the flywheel at that speed (e.g. while driving or aiming); a volley spins
 *    it anyway.
 *  - startVolley(): one ball per indexer step, INDEXER_STEPS_PER_VOLLEY steps (4 balls + 1 spare):
 *    waits for the flywheel to be at speed (open-loop: OPEN_LOOP_SPIN_UP_MS after switching on),
 *    then after each step SHOT_INTERVAL_MS and the flywheel back at speed (as in the Mechanism Test).
 *    The indexer handles jams itself (back off, retry); a STUCK indexer ends the volley (stuck()).
 *  - setPaused(true) holds the next ball (e.g. while knocked off aim); a step already started
 *    finishes.
 */
@Config("JohnShooter")
public class Shooter {

    /**
     * [John] Shot Calibration 2026-11-01 and 11-02 (r0273-r0278, 16 spots, F 18.76), distances from
     * odometry to the nominal CELL (ShootingZone.shotDistance): the middle of the speeds that scored at
     * least 90%, averaged over the angles at each distance (no angle effect showed: at 50" 15 / 30 / 45
     * deg gave 2300 / 2288 / 2288). 42" 2225-2275 (0-30 deg; at 45 deg nothing reached 60%), 46" 2275,
     * 49" 2275, 50" 2290, 54" 2320, 58" 2375 (83%), 66" 2425 (84%; the farthest the field allows).
     * John v2 (gear belt, F 23.31): Shot Calibration r1071, 2026-11-14 (100 volleys, 13 spots), each
     * volley from a fresh spin-up. 50-100 RPM below v1 (the old slip belt let the flywheel lag the
     * motor). Windows (>= 90%): 48.5"/0 2150-2350; 50"/15 2175-2325, /30 2200, /38 2200-2250, /45
     * 2150-2200; 54"/30 2225-2275, /38 2225, /45 2275; 58"/38 2225-2275, /45 2275-2325; 62"/45 2350;
     * 42"/0 2250 (90%), /30 2250 (79%). 45 deg needs ~25-50 more than 38 at 54-58". 46" interpolated,
     * 66" extrapolated (not tested).
     */
    public static String RPM_DISTANCES_IN = "42,46,50,54,58,62,66";
    public static String RPM_VALUES = "2250,2225,2200,2250,2275,2350,2400";
    public static double FIRE_TOLERANCE_NEAR_RPM = 100;
    public static double FIRE_TOLERANCE_FAR_RPM = 50;
    /**
     * ... but at most this much ABOVE the target. Shot Calibration (r1071) fired each volley from a
     * fresh spin-up, the balls leaving at or just under the set speed; in Auto r1072 / r1076 volley 2
     * the flywheel was already up and still overshooting (2271-2314 for 2200), and half the balls went
     * long. Readings come in ~43 RPM steps, so there is always one within the window.
     */
    public static double FIRE_ABOVE_RPM = 30;
    /**
     * ... unless it has waited this long above that window (still within the normal tolerance): a
     * slightly fast ball beats a stalled volley (Auto r1083: no ball left to pull it down, the flywheel
     * sat 70-115 fast and the volley hung 5.9 s on its spare step).
     */
    public static double FIRE_ABOVE_WAIT_MS = 500;
    /**
     * Steps past this many in a volley are spares (for a ball that comes late): they fire without the
     * upper speed limit. With no ball left to pull it down the flywheel runs 100-150 RPM fast, and the
     * spare step waited 2.2 s for it (Auto r1152).
     */
    public static int BALLS_PER_VOLLEY = 4;
    /**
     * The first ball of a volley: the flywheel aims this far from the table speed and the ball fires
     * once within FIRST_BALL_BELOW_RPM under that (at most FIRE_ABOVE_RPM over); from the second ball
     * on, the table speed. Shot Calibration's first balls left about 50 RPM slower than Auto's (each
     * volley there fired right at the hand-off from the boost, ~100 under the set speed, and the
     * flywheel was still climbing as the ball went through); the first ball of volley 1 went long in
     * r1149 and r1152 (fired at 2143, the flywheel 2228-2271 as it went through, target 2208).
     */
    public static double FIRST_BALL_OFFSET_RPM = -50;
    public static double FIRST_BALL_BELOW_RPM = 40;
    /**
     * While only spinning (setSpin, no volley) the flywheel waits this far from the table speed; the
     * volley then raises it to the table speed. Shot Calibration (r1071) spun up from rest for every
     * volley, so its balls left while the speed was still rising: 29-93 RPM under the set speed
     * (median 61, 491 balls). A flywheel spun up early and settled at the set speed (Auto volley 2,
     * TeleOp pre-spin) shot ~60-130 faster: r1088's first ball of volley 2 went long at 2229-2271
     * for 2200.
     */
    public static double PRESPIN_OFFSET_RPM = -60;
    /**
     * ... and the volley raises it to the table speed at this rate, not in one step: the 60 RPM jump
     * with P 25 made the flywheel overshoot to 70-110 over the target once ball 1 was gone, and ball 2
     * waited up to 1.1 s for the firing window (Auto r1134). Ball 1 fires as before (within the
     * tolerance under the target). Lowering is immediate.
     */
    public static double PRESPIN_RAMP_RPM_PER_S = 1e9;   // off (instant) in TeleOp; JohnAuto sets AUTO_PRESPIN_RAMP_RPM_PER_S (150) for its run
    public static double TOLERANCE_NEAR_IN = 52;
    public static double TOLERANCE_FAR_IN = 60;
    /**
     * 4 balls, plus 1 extra indexer step in case a ball comes late (was 2 extra; each empty step
     * costs ~0.6 s, and the GARDEN gives at most 4 balls).
     */
    public static int INDEXER_STEPS_PER_VOLLEY = 5;
    /** Time between two balls (200-250 ms worked in the Mechanism Test; 150 ms missed 1 in 4). */
    public static double SHOT_INTERVAL_MS = 250;
    /** Flywheel without its encoder (open-loop): wait this long after switching it on. */
    public static double OPEN_LOOP_SPIN_UP_MS = 1500;
    /** Long enough for the indexer's jam recovery (see Indexer: 2 tries, about 2.4 s). Was 4000. */
    public static double STEP_TIMEOUT_MS = 3000;
    /**
     * A step this long that is within STEP_CLOSE_DEG of its target counts as done (2026-10-31, r0240:
     * an empty step crept the last degrees for 3.6 s and cost the second volley). The next step's
     * target is planned from the step count, so nothing is lost.
     */
    public static double STEP_SLOW_MS = 800;
    public static double STEP_CLOSE_DEG = 20;

    private enum Feed { IDLE, SPIN_UP, STEP, INTERVAL }

    private final Flywheel flywheel;
    private final Indexer indexer;
    private final ElapsedTime feedTimer = new ElapsedTime();
    /** How long the flywheel has been above the firing window but within the normal tolerance. */
    private final ElapsedTime fastTimer = new ElapsedTime();   // loop timer for the above-window wait
    /** Time spent above the firing window since the last ball (cumulative: the readings jump ~43 RPM). */
    private double aboveMs;
    /** The flywheel target as ramped (PRESPIN_RAMP_RPM_PER_S); NaN before the first. */
    private double rampedRpm = Double.NaN;
    private final ElapsedTime rampTimer = new ElapsedTime();
    private boolean spin;
    private boolean paused;
    private Feed feed = Feed.IDLE;
    private int steps;
    private int stepsDone;
    private int volleys;
    private boolean stuck;
    private double distance = Double.NaN;
    private double trimRpm;
    private Double rpmOverride;
    /** The speed for the distance (table or override, plus the trim), set by setDistance(). */
    private double baseRpm = Double.NaN;
    private String status = "idle";
    private String event;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = new Flywheel(hardwareMap);
        indexer = new Indexer(hardwareMap);
    }

    /** Keep the flywheel spinning at the speed for the distance (a volley spins it anyway). */
    public void setSpin(boolean on) {
        spin = on;
    }

    /** Distance to the CELL (inches): sets the flywheel speed (plus the trim), except during the feeding. */
    public void setDistance(double inches) {
        distance = inches;
        if (feed == Feed.IDLE || feed == Feed.SPIN_UP) {
            baseRpm = rpmOverride != null ? rpmOverride : rpmFor(inches) + trimRpm;
        }
    }

    /** A fixed speed instead of the table (null: the table again); setDistance() still sets the tolerance. */
    public void setRpmOverride(Double rpm) {
        rpmOverride = rpm;
    }

    /** The fire tolerance for a distance (FIRE_TOLERANCE_NEAR_RPM / FAR_RPM; near without a distance). */
    public static double fireToleranceRpm(double distance) {
        if (Double.isNaN(distance) || distance <= TOLERANCE_NEAR_IN) {
            return FIRE_TOLERANCE_NEAR_RPM;
        }
        if (distance >= TOLERANCE_FAR_IN) {
            return FIRE_TOLERANCE_FAR_RPM;
        }
        double f = (distance - TOLERANCE_NEAR_IN) / (TOLERANCE_FAR_IN - TOLERANCE_NEAR_IN);
        return FIRE_TOLERANCE_NEAR_RPM + f * (FIRE_TOLERANCE_FAR_RPM - FIRE_TOLERANCE_NEAR_RPM);
    }

    /** Driver's correction added to the table speed (e.g. for the battery). */
    public void setTrimRpm(double rpm) {
        trimRpm = rpm;
    }

    public double trimRpm() {
        return trimRpm;
    }

    public void startVolley() {
        startVolley(INDEXER_STEPS_PER_VOLLEY);
    }

    public void startVolley(int indexerSteps) {
        feed = Feed.SPIN_UP;
        steps = Math.max(1, indexerSteps);
        stepsDone = 0;
        stuck = false;
        status = "waiting for the flywheel";
    }

    /** Hold the next ball while true (the volley goes on when false again). */
    public void setPaused(boolean paused) {
        this.paused = paused;
    }

    public void cancel() {
        if (feed != Feed.IDLE) {
            status = "volley cancelled";
        }
        feed = Feed.IDLE;
    }

    /** A volley is running. */
    public boolean firing() {
        return feed != Feed.IDLE;
    }

    /** Indexer steps of the current / last volley not done (e.g. after cancel()). */
    public int stepsLeft() {
        return Math.max(0, steps - stepsDone);
    }

    /** The last volley ended because the indexer was stuck (clear the ball). */
    public boolean stuck() {
        return stuck;
    }

    /** Flywheel, volley and indexer for this loop. Returns what happened (or null). Call every loop. */
    public String update() {
        event = null;
        if (!Double.isNaN(baseRpm)) {
            double desired = feed == Feed.IDLE ? baseRpm + PRESPIN_OFFSET_RPM
                    : feed == Feed.SPIN_UP && stepsDone == 0 ? baseRpm + FIRST_BALL_OFFSET_RPM : baseRpm;
            double dt = Math.min(0.1, rampTimer.seconds());
            rampTimer.reset();
            // Up gradually only from a spinning flywheel; from rest (or down) straight to the speed
            if (Double.isNaN(rampedRpm) || !flywheel.isOn() || desired <= rampedRpm) {
                rampedRpm = desired;
            } else {
                rampedRpm = Math.min(desired, rampedRpm + PRESPIN_RAMP_RPM_PER_S * dt);
            }
            flywheel.setTargetRpm(rampedRpm);
        }
        flywheel.setOn(spin || feed != Feed.IDLE);
        flywheel.update();
        add(flywheel.event());
        updateFeed();
        indexer.update();
        add(indexer.event());
        return event;
    }

    private void updateFeed() {
        if (feed != Feed.IDLE && flywheel.stalled()) {
            feed = Feed.IDLE;
            volleys++;
            status = String.format(Locale.US, "SKIPPED after %d / %d: the flywheel does not turn", stepsDone, steps);
            add("volley skipped: flywheel stalled");
            return;
        }
        double tolerance = fireToleranceRpm(distance);
        boolean firstBall = feed == Feed.SPIN_UP && stepsDone == 0;
        boolean inWindow = firstBall
                ? flywheel.within(FIRST_BALL_BELOW_RPM, FIRE_ABOVE_RPM)   // around the lowered first-ball target
                : flywheel.within(tolerance, Math.min(FIRE_ABOVE_RPM, tolerance));
        boolean above = !inWindow && flywheel.rpm() > flywheel.targetRpm();
        double loopMs = Math.min(100.0, fastTimer.milliseconds());
        fastTimer.reset();
        if (inWindow || feed == Feed.IDLE) {
            aboveMs = 0.0;
        } else if (above) {
            aboveMs += loopMs;   // a reading past the tolerance does not start the wait over
        }
        boolean fastButClose = above && flywheel.within(tolerance);
        boolean spare = stepsDone >= BALLS_PER_VOLLEY && flywheel.within(tolerance, Double.POSITIVE_INFINITY);
        boolean ready = !paused
                && (flywheel.closedLoop() ? inWindow || spare || (fastButClose && aboveMs >= FIRE_ABOVE_WAIT_MS)
                : flywheel.onSeconds() * 1000.0 >= OPEN_LOOP_SPIN_UP_MS);
        switch (feed) {
            case SPIN_UP:
                if (ready) {
                    step();
                }
                break;
            case STEP:
                if (indexer.stuck()) {
                    feed = Feed.IDLE;
                    stuck = true;
                    volleys++;
                    status = String.format(Locale.US, "STOPPED after %d / %d: indexer stuck, clear the ball", stepsDone, steps);
                    add("volley stopped: indexer stuck");
                    break;
                }
                // busy() needs a moment to come on after a new target
                double shortDeg = indexer.targetDegrees() - indexer.degrees();
                boolean slowButClose = feedTimer.milliseconds() > STEP_SLOW_MS && Math.abs(shortDeg) < STEP_CLOSE_DEG
                        && indexer.busy();
                if (slowButClose) {
                    add(String.format(Locale.US, "indexer step slow: %.0f deg short after %.0f ms, moving on",
                            shortDeg, feedTimer.milliseconds()));
                }
                // Done once the step got there (Indexer.arrived(): it overshoots, then creeps back for
                // 0.4-0.5 s with the ball already through), or the motor stopped
                if ((feedTimer.milliseconds() > 50 && (indexer.arrived() || !indexer.busy())) || slowButClose
                        || feedTimer.milliseconds() > STEP_TIMEOUT_MS) {
                    stepsDone++;
                    if (stepsDone >= steps) {
                        feed = Feed.IDLE;
                        volleys++;
                        status = "volley done";
                        add("volley done");
                    } else {
                        feed = Feed.INTERVAL;
                        feedTimer.reset();
                    }
                }
                break;
            case INTERVAL:
                if (feedTimer.milliseconds() >= SHOT_INTERVAL_MS && ready) {
                    step();
                }
                break;
            default:
                break;
        }
    }

    private void step() {
        aboveMs = 0.0;
        indexer.step(1);
        feed = Feed.STEP;
        feedTimer.reset();
        status = String.format(Locale.US, "shot %d / %d", stepsDone + 1, steps);
        add(String.format(Locale.US, "shot %d at %.0f RPM", stepsDone + 1, flywheel.rpm()));
    }

    private void add(String e) {
        if (e != null) {
            event = event == null ? e : event + "; " + e;
        }
    }

    /** Fires a volley (the flywheel spins up first); done when it is over. */
    public Command volley() {
        return new CommandBuilder()
                .setStart(this::startVolley)
                .setDone(() -> !firing());
    }

    public void stop() {
        feed = Feed.IDLE;
        spin = false;
        flywheel.stop();
        indexer.stop();
    }

    public Flywheel flywheel() {
        return flywheel;
    }

    public Indexer indexer() {
        return indexer;
    }

    public int volleys() {
        return volleys;
    }

    public String status() {
        return String.format(Locale.US, "%s%s; flywheel %s %.0f / %.0f RPM%s%s", status,
                paused && feed != Feed.IDLE ? " (PAUSED)" : "", flywheel.isOn() ? "ON" : "off",
                flywheel.rpm(), flywheel.targetRpm(), flywheel.atSpeed() ? " AT SPEED" : "",
                Double.isNaN(distance) || rpmInRange(distance) ? "" : " (distance outside the tested range)");
    }

    /** Flywheel RPM for a distance: linear between the table points, clamped at its ends. */
    public static double rpmFor(double distance) {
        double[] d = parse(RPM_DISTANCES_IN);
        double[] r = parse(RPM_VALUES);
        int n = Math.min(d.length, r.length);
        if (n == 0) {
            return Flywheel.DEFAULT_TARGET_RPM;
        }
        if (distance <= d[0]) {
            return r[0];
        }
        for (int i = 1; i < n; i++) {
            if (distance <= d[i]) {
                return r[i - 1] + (r[i] - r[i - 1]) * (distance - d[i - 1]) / (d[i] - d[i - 1]);
            }
        }
        return r[n - 1];
    }

    /** Within 3" of the tested distances. */
    public static boolean rpmInRange(double distance) {
        double[] d = parse(RPM_DISTANCES_IN);
        return d.length == 0 || (distance >= d[0] - 3.0 && distance <= d[d.length - 1] + 3.0);
    }

    private static double[] parse(String list) {
        String[] parts = list.split(",");
        double[] out = new double[parts.length];
        int n = 0;
        for (String p : parts) {
            try {
                out[n] = Double.parseDouble(p.trim());
                n++;
            } catch (NumberFormatException ignored) {
                // skip
            }
        }
        double[] trimmed = new double[n];
        System.arraycopy(out, 0, trimmed, 0, n);
        return trimmed;
    }
}
