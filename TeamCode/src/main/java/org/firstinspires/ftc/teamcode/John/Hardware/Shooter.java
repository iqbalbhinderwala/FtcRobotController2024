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
 *  - startVolley(): one ball per indexer step, INDEXER_STEPS_PER_VOLLEY steps (4 balls + 2 spare):
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
     */
    public static String RPM_DISTANCES_IN = "42,46,49,50,54,58,66";
    public static String RPM_VALUES = "2250,2275,2275,2290,2320,2375,2425";
    public static double FIRE_TOLERANCE_NEAR_RPM = 100;
    public static double FIRE_TOLERANCE_FAR_RPM = 50;
    public static double TOLERANCE_NEAR_IN = 52;
    public static double TOLERANCE_FAR_IN = 60;
    /** 4 balls, plus 2 extra indexer steps in case a ball comes late. */
    public static int INDEXER_STEPS_PER_VOLLEY = 6;
    /** Time between two balls (200-250 ms worked in the Mechanism Test; 150 ms missed 1 in 4). */
    public static double SHOT_INTERVAL_MS = 250;
    /** Flywheel without its encoder (open-loop): wait this long after switching it on. */
    public static double OPEN_LOOP_SPIN_UP_MS = 1500;
    /** Long enough for the indexer's jam recovery (see Indexer). */
    public static double STEP_TIMEOUT_MS = 4000;
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
            flywheel.setTargetRpm(rpmOverride != null ? rpmOverride : rpmFor(inches) + trimRpm);
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
        flywheel.setOn(spin || feed != Feed.IDLE);
        flywheel.update();
        add(flywheel.event());
        updateFeed();
        indexer.update();
        add(indexer.event());
        return event;
    }

    private void updateFeed() {
        boolean ready = !paused
                && (flywheel.closedLoop() ? flywheel.within(fireToleranceRpm(distance))
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
                if ((feedTimer.milliseconds() > 50 && !indexer.busy()) || slowButClose
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
