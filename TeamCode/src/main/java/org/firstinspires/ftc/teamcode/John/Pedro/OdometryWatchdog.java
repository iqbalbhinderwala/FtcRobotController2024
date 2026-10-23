package org.firstinspires.ftc.teamcode.John.Pedro;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

/**
 * Keeps the drive from acting on bad Pinpoint odometry (a frozen or reset heading during a hold makes
 * the robot spin forever).
 *
 * Call check(follower) every loop right after follower.update(). Each Pinpoint read is judged:
 *  - good: status READY, a sane Pinpoint loop time, and a pose that is continuous with the last good
 *    pose (it cannot have moved / turned faster than MAX_SPEED_IN_S / MAX_TURN_DEG_S allow);
 *  - bad: any other status (NOT_READY, CALIBRATING, pod faults), a garbage loop time, a pose jump,
 *    a pose near the field center (see below), or FAULT_BAD_READ for BAD_READ_LOOPS loops in a row
 *    (single bad reads are normal: the driver keeps the last good value).
 *
 * Failed I2C reads come back as zeros or garbage, but the Pinpoint keeps tracking internally, so
 * once reads succeed again the pose is right (seen twice: 0.15 s and 0.47 s, both resumed on the
 * true track). A Pinpoint that really rebooted restarts at (0, 0, 0): in FTC coordinates that is
 * the field center, inside the HIVE, where John can never be. So a pose within REBOOT_RADIUS_IN of
 * the center with a heading within REBOOT_HEADING_DEG of 0, after being elsewhere, is a bad read or
 * a reboot, at any pause length.
 *
 * States:
 *  - OK: nothing to do.
 *  - PAUSED: after a bad read the drive motors are set to zero power every loop (not braked), but
 *    the follower keeps its command (hold / path), so it carries on by itself after RESUME_LOOPS
 *    good reads in a row.
 *  - LOST: still not resumed after MAX_PAUSE_MS (a reboot that stays at the field center, or a dead
 *    I2C link). Latched: the follower is stopped for good; restart the OpMode after fixing the
 *    hardware.
 *
 * It reads the same Pinpoint instance as Pedro's PinpointLocalizer (hardwareMap returns one object
 * per device), whose update() refreshes the status.
 *
 * Event log: every bad read, PAUSED, RESUMED and LOST goes to its own file,
 * /sdcard/FIRST/john_logs/r<run>_john_odometry_<opmode>_<time>.csv, with the Pinpoint status, loop
 * time, the pose and the last good pose, and the speed and turn rate at the last good read (just
 * before a dropout). The file is only created at the first event (so its mere existence means
 * something happened) and every row is flushed at once (nothing is lost if the app is killed).
 *
 * To tell whether dropouts come with fast driving or turning, it also counts the time spent in
 * speed and turn-rate bands (SPEED_BANDS_IPS, TURN_BANDS_DPS) and the pauses that started in each;
 * close() writes that as "exposure" rows when the OpMode ends. Call close() when the OpMode ends.
 */
@Config
public class OdometryWatchdog {

    public enum State { OK, PAUSED, LOST }

    public static int BAD_READ_LOOPS = 5;
    public static int RESUME_LOOPS = 3;
    /**
     * Longest pause before LOST. Glitches seen so far lasted 0.15 s and 0.47 s, and resumed on the
     * true pose. A reboot is caught by the field-center check at any length, so this can be long.
     */
    public static double MAX_PAUSE_MS = 2500.0;
    /** A pose this close to the field center (inside the HIVE) with a heading near 0 is a reboot. */
    public static double REBOOT_RADIUS_IN = 6.0;
    public static double REBOOT_HEADING_DEG = 10.0;
    /** Pinpoint internal loop time considered sane (normally about 500-1100 us). */
    public static int MIN_LOOP_US = 200;
    public static int MAX_LOOP_US = 5000;
    /** Continuity limits between good reads (generous: well above what the robot can do). */
    public static double MAX_SPEED_IN_S = 120.0;
    public static double MAX_TURN_DEG_S = 900.0;
    public static double JUMP_MARGIN_IN = 1.0;
    public static double JUMP_MARGIN_DEG = 5.0;
    /** Lower edges of the speed / turn-rate bands for the exposure rows. */
    private static final double[] SPEED_BANDS_IPS = {0, 10, 30, 50, 70};
    private static final double[] TURN_BANDS_DPS = {0, 45, 90, 180, 360};

    private final GoBildaPinpointDriver pinpoint;
    private final ElapsedTime clock = new ElapsedTime();

    private State state = State.OK;
    private String reason = "";
    private Pose lastGood;
    private double lastGoodS;
    private double pausedAtS;
    private double lastPauseMs;
    private int badReads;
    private int goodStreak;
    private int pauses;

    private static final String LOG_HEADER = "t_s,event,state,pinpoint_status,pinpoint_loop_us,x,y,h_deg,"
            + "last_good_x,last_good_y,last_good_h_deg,since_last_good_ms,pause_ms,"
            + "last_good_speed_ips,last_good_turn_dps,reason";
    private final String logName;
    private CsvLog log;

    /** Speed and turn rate at the last good read (the Pinpoint's own velocity). */
    private double lastGoodSpeed;
    private double lastGoodTurnRate;
    /** Time (s) spent OK in each band, and pauses that started in each band. */
    private final double[] speedTime = new double[SPEED_BANDS_IPS.length];
    private final double[] turnTime = new double[TURN_BANDS_DPS.length];
    private final int[] speedPauses = new int[SPEED_BANDS_IPS.length];
    private final int[] turnPauses = new int[TURN_BANDS_DPS.length];
    private double lastCheckS = Double.NaN;

    /** @param opModeName short name for the event log file, e.g. "auto" or "hold_test" */
    public OdometryWatchdog(HardwareMap hardwareMap, String opModeName) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        logName = "john_odometry_" + opModeName;
    }

    /** @return the state after this loop's read; the drive is stopped unless it is OK */
    public State check(Follower follower) {
        if (state == State.LOST) {
            stopForGood(follower);
            return state;
        }
        double now = clock.seconds();
        double dt = Double.isNaN(lastCheckS) ? 0.0 : now - lastCheckS;
        lastCheckS = now;
        Pose pose = follower.pose();
        String problem = judge(pose, now);

        if (problem.isEmpty()) {
            if (state == State.PAUSED) {
                logEvent(now, "good_read", pose, "");   // the good reads that lead to RESUMED
            }
            lastGood = pose;
            lastGoodS = now;
            Velocity v = follower.velocity();
            if (v != null) {
                lastGoodSpeed = Math.hypot(v.vx, v.vy);
                lastGoodTurnRate = Math.abs(Math.toDegrees(v.omega));
            }
            if (state == State.OK) {
                speedTime[band(SPEED_BANDS_IPS, lastGoodSpeed)] += dt;
                turnTime[band(TURN_BANDS_DPS, lastGoodTurnRate)] += dt;
            }
            if (state == State.PAUSED && ++goodStreak >= RESUME_LOOPS) {
                state = State.OK;
                lastPauseMs = (now - pausedAtS) * 1000.0;
                logEvent(now, "RESUMED", pose, String.format("after %.0f ms", lastPauseMs));
            }
        } else {
            goodStreak = 0;
            if (state == State.OK) {
                state = State.PAUSED;
                reason = problem;
                pausedAtS = now;
                pauses++;
                speedPauses[band(SPEED_BANDS_IPS, lastGoodSpeed)]++;
                turnPauses[band(TURN_BANDS_DPS, lastGoodTurnRate)]++;
                logEvent(now, "PAUSED", pose, problem);
            } else {
                logEvent(now, "bad_read", pose, problem);
            }
        }

        if (state == State.PAUSED) {
            if ((now - pausedAtS) * 1000.0 > MAX_PAUSE_MS) {
                state = State.LOST;
                reason = reason + "; not recovered after " + Math.round(MAX_PAUSE_MS) + " ms";
                logEvent(now, "LOST", pose, reason);
                stopForGood(follower);
            } else {
                follower.drivetrain.stop();   // keep the follower's command so it resumes by itself
            }
        }
        return state;
    }

    /** Call after follower.setPose(...), so the deliberate jump is not taken for a bad read. */
    public void poseSet() {
        lastGood = null;
    }

    /** "" when this read is good, else why not. */
    private String judge(Pose pose, double now) {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            // The driver kept the last good values: stale but sane, unless it keeps happening
            return ++badReads >= BAD_READ_LOOPS ? "Pinpoint bad I2C reads for " + badReads + " loops" : "";
        }
        badReads = 0;
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            return "Pinpoint status " + status;
        }
        int loopUs = pinpoint.getLoopTime();
        if (loopUs < MIN_LOOP_US || loopUs > MAX_LOOP_US) {
            return "Pinpoint loop time " + loopUs + " us";
        }
        if (Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            return "Pinpoint pose NaN";
        }
        if (lastGood != null) {
            // At the field center with heading ~0 after being elsewhere: a zero read or a reboot
            boolean nearOrigin = Math.hypot(pose.x(), pose.y()) < REBOOT_RADIUS_IN
                    && Math.abs(Math.toDegrees(normalize(pose.heading()))) < REBOOT_HEADING_DEG;
            if (nearOrigin && Math.hypot(lastGood.x(), lastGood.y()) >= REBOOT_RADIUS_IN) {
                return String.format("Pinpoint pose at the field center (%.1f, %.1f, %.0f deg): zero read or reboot",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            }
            double dt = now - lastGoodS;
            double moved = Math.hypot(pose.x() - lastGood.x(), pose.y() - lastGood.y());
            double turned = Math.abs(Math.toDegrees(normalize(pose.heading() - lastGood.heading())));
            if (moved > MAX_SPEED_IN_S * dt + JUMP_MARGIN_IN || turned > MAX_TURN_DEG_S * dt + JUMP_MARGIN_DEG) {
                return String.format("Pinpoint pose jumped %.1f in / %.0f deg in %.0f ms", moved, turned, dt * 1000.0);
            }
        }
        return "";
    }

    /** One row in the event log (created at the first event), flushed at once. */
    private void logEvent(double now, String event, Pose pose, String detail) {
        if (log == null) {
            log = new CsvLog(logName, LOG_HEADER);
        }
        log.row(now, event, state, pinpoint.getDeviceStatus(), pinpoint.getLoopTime(),
                pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                lastGood == null ? null : lastGood.x(), lastGood == null ? null : lastGood.y(),
                lastGood == null ? null : Math.toDegrees(lastGood.heading()),
                lastGood == null ? null : (now - lastGoodS) * 1000.0,
                state == State.OK ? null : (now - pausedAtS) * 1000.0, lastGoodSpeed, lastGoodTurnRate, detail);
        log.flush();
    }

    /** Index of the band a value falls in (bands given by their lower edges). */
    private static int band(double[] lowerEdges, double value) {
        int i = 0;
        while (i + 1 < lowerEdges.length && value >= lowerEdges[i + 1]) {
            i++;
        }
        return i;
    }

    private static String bandLabel(double[] lowerEdges, int i, String unit) {
        return i + 1 < lowerEdges.length
                ? String.format("%.0f-%.0f %s", lowerEdges[i], lowerEdges[i + 1], unit)
                : String.format("%.0f+ %s", lowerEdges[i], unit);
    }

    /**
     * Closes the event log, if any event happened, after writing the exposure rows: the time spent
     * in each speed / turn-rate band and the pauses that started in it. Call when the OpMode ends.
     */
    public void close() {
        if (log == null) {
            return;
        }
        double now = clock.seconds();
        for (int i = 0; i < SPEED_BANDS_IPS.length; i++) {
            log.row(now, "exposure", null, null, null, null, null, null, null, null, null, null, null, null, null,
                    String.format("speed %s: %.1f s driving, %d pause(s)",
                            bandLabel(SPEED_BANDS_IPS, i, "in/s"), speedTime[i], speedPauses[i]));
        }
        for (int i = 0; i < TURN_BANDS_DPS.length; i++) {
            log.row(now, "exposure", null, null, null, null, null, null, null, null, null, null, null, null, null,
                    String.format("turn %s: %.1f s, %d pause(s)",
                            bandLabel(TURN_BANDS_DPS, i, "deg/s"), turnTime[i], turnPauses[i]));
        }
        log.close();
    }

    /** Name of the event log file, or "" if nothing has happened. */
    public String logFileName() {
        return log == null ? "" : log.fileName();
    }

    private static void stopForGood(Follower follower) {
        follower.stop();
        follower.drivetrain.stop();
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    public State state() {
        return state;
    }

    public boolean ok() {
        return state == State.OK;
    }

    public boolean lost() {
        return state == State.LOST;
    }

    /** Why it last paused (and, when LOST, that it did not recover). */
    public String reason() {
        return reason;
    }

    /** How long the last completed pause lasted. */
    public double lastPauseMs() {
        return lastPauseMs;
    }

    public int pauses() {
        return pauses;
    }

    public GoBildaPinpointDriver.DeviceStatus status() {
        return pinpoint.getDeviceStatus();
    }

    /** Pinpoint internal loop time in microseconds (normally about 500-1100). */
    public int pinpointLoopUs() {
        return pinpoint.getLoopTime();
    }
}
