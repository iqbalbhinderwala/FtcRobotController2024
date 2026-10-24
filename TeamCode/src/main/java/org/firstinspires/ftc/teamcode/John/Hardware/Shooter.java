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
 *    between, clamped at the ends. The points are the 2026-10-23 tests shooting at the up CELL from
 *    the audience wall: about 2200 RPM from C1, 2300 from B1, 2400 from A1, at 50.3 / 55.5 / 69.0"
 *    (robot center to the CELL's tag cluster center, from above; see ShootingZone).
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

    public static String RPM_DISTANCES_IN = "50.3,55.5,69.0";
    public static String RPM_VALUES = "2200,2300,2400";
    /** 4 balls, plus 2 extra indexer steps in case a ball comes late. */
    public static int INDEXER_STEPS_PER_VOLLEY = 6;
    /** Time between two balls (200-250 ms worked in the Mechanism Test; 150 ms missed 1 in 4). */
    public static double SHOT_INTERVAL_MS = 250;
    /** Flywheel without its encoder (open-loop): wait this long after switching it on. */
    public static double OPEN_LOOP_SPIN_UP_MS = 1500;
    /** Long enough for the indexer's jam recovery (see Indexer). */
    public static double STEP_TIMEOUT_MS = 4000;

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
            flywheel.setTargetRpm(rpmFor(inches) + trimRpm);
        }
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
                && (flywheel.closedLoop() ? flywheel.atSpeed() : flywheel.onSeconds() * 1000.0 >= OPEN_LOOP_SPIN_UP_MS);
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
                if ((feedTimer.milliseconds() > 50 && !indexer.busy()) || feedTimer.milliseconds() > STEP_TIMEOUT_MS) {
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
