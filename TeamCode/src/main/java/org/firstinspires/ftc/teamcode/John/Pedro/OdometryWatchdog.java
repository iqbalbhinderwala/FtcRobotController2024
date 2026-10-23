package org.firstinspires.ftc.teamcode.John.Pedro;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Keeps the drive from acting on bad Pinpoint odometry (a frozen or reset heading during a hold makes
 * the robot spin forever).
 *
 * Call check(follower) every loop right after follower.update(). Each Pinpoint read is judged:
 *  - good: status READY, a sane Pinpoint loop time, and a pose that is continuous with the last good
 *    pose (it cannot have moved / turned faster than MAX_SPEED_IN_S / MAX_TURN_DEG_S allow);
 *  - bad: any other status (NOT_READY, CALIBRATING, pod faults), a garbage loop time, a pose jump,
 *    a pose of exactly (0, 0) after being elsewhere (reboot), or FAULT_BAD_READ for BAD_READ_LOOPS
 *    loops in a row (single
 *    bad reads are normal: the driver keeps the last good value).
 *
 * States:
 *  - OK: nothing to do.
 *  - PAUSED: after a bad read the drive motors are stopped every loop, but the follower keeps its
 *    command (hold / path), so it carries on by itself after RESUME_LOOPS good reads in a row. This
 *    rides through short I2C glitches (seen: ~0.15 s of corrupt reads while spinning hard).
 *  - LOST: still not resumed after MAX_PAUSE_MS (a reboot that lost the pose, or a dead I2C link).
 *    Latched: the follower is stopped for good; restart the OpMode after fixing the hardware.
 *
 * It reads the same Pinpoint instance as Pedro's PinpointLocalizer (hardwareMap returns one object
 * per device), whose update() refreshes the status.
 */
@Config
public class OdometryWatchdog {

    public enum State { OK, PAUSED, LOST }

    public static int BAD_READ_LOOPS = 5;
    public static int RESUME_LOOPS = 3;
    /**
     * Longest pause before LOST. Glitches seen so far lasted 0.15 s and 0.47 s. Longer pauses weaken
     * the continuity check on the resumed pose (it allows MAX_SPEED_IN_S x the pause), so keep this
     * around a second.
     */
    public static double MAX_PAUSE_MS = 1000.0;
    /** Pinpoint internal loop time considered sane (normally about 500-1100 us). */
    public static int MIN_LOOP_US = 200;
    public static int MAX_LOOP_US = 5000;
    /** Continuity limits between good reads (generous: well above what the robot can do). */
    public static double MAX_SPEED_IN_S = 120.0;
    public static double MAX_TURN_DEG_S = 900.0;
    public static double JUMP_MARGIN_IN = 1.0;
    public static double JUMP_MARGIN_DEG = 5.0;

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

    public OdometryWatchdog(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
    }

    /** @return the state after this loop's read; the drive is stopped unless it is OK */
    public State check(Follower follower) {
        if (state == State.LOST) {
            stopForGood(follower);
            return state;
        }
        double now = clock.seconds();
        Pose pose = follower.pose();
        String problem = judge(pose, now);

        if (problem.isEmpty()) {
            lastGood = pose;
            lastGoodS = now;
            if (state == State.PAUSED && ++goodStreak >= RESUME_LOOPS) {
                state = State.OK;
                lastPauseMs = (now - pausedAtS) * 1000.0;
            }
        } else {
            goodStreak = 0;
            if (state == State.OK) {
                state = State.PAUSED;
                reason = problem;
                pausedAtS = now;
                pauses++;
            }
        }

        if (state == State.PAUSED) {
            if ((now - pausedAtS) * 1000.0 > MAX_PAUSE_MS) {
                state = State.LOST;
                reason = reason + "; not recovered after " + Math.round(MAX_PAUSE_MS) + " ms";
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
            // Exactly zero after being elsewhere: the Pinpoint reset (rebooted) and lost the pose
            if (pose.x() == 0.0 && pose.y() == 0.0 && Math.hypot(lastGood.x(), lastGood.y()) > JUMP_MARGIN_IN) {
                return "Pinpoint pose reset to (0, 0)";
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
