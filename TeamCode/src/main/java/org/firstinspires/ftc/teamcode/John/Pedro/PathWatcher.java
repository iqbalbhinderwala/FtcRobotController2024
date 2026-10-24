package org.firstinspires.ftc.teamcode.John.Pedro;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Watches the follower while it drives a path, for another robot (or a wall) in the way. John has no
 * distance sensors, so it goes by what the odometry says:
 *  - BLOCKED: slower than BLOCKED_SPEED_IPS (and turning slower than BLOCKED_TURN_DPS) for
 *    BLOCKED_MS while more than MIN_REMAINING_IN of the path is left (Pedro's remainingDistance:
 *    of the current leg, so not near a leg's end): something stops it;
 *  - PUSHED: more than PUSHED_IN from the path for PUSHED_MS: something shoves it aside.
 * Not in the first START_GRACE_MS of a path (it is still speeding up).
 *
 * follow(): drives a path and, when it trips, stops pushing (zero power, braking), waits
 * RETRY_WAIT_MS (the other robot may just be passing), plans the path again from where the robot
 * is and retries, up to RETRIES times; then gives up, with the robot stopped (gaveUp() is true
 * until the next follow()). The caller decides what to do then.
 *
 * Every trip, retry and give-up goes to the log callback.
 */
@Config("JohnPathWatcher")
public class PathWatcher {

    public static boolean ENABLED = true;
    public static double BLOCKED_SPEED_IPS = 2.0;
    public static double BLOCKED_TURN_DPS = 15.0;
    public static double BLOCKED_MS = 500;
    public static double MIN_REMAINING_IN = 3.0;
    public static double PUSHED_IN = 5.0;
    public static double PUSHED_MS = 200;
    public static double START_GRACE_MS = 400;
    public static int RETRIES = 2;
    public static double RETRY_WAIT_MS = 1000;

    public enum Trip { NONE, BLOCKED, PUSHED }

    private final Follower follower;
    private final Consumer<String> log;
    private final ElapsedTime timer = new ElapsedTime();
    private double slowSinceMs = -1;
    private double offSinceMs = -1;
    private String reason = "";
    private int trips;
    private boolean gaveUp;

    public PathWatcher(Follower follower, Consumer<String> log) {
        this.follower = follower;
        this.log = log;
    }

    /** Starts watching a new path. */
    public void start() {
        timer.reset();
        slowSinceMs = -1;
        offSinceMs = -1;
    }

    /** Call every loop while a path is followed (after follower.update()). */
    public Trip check() {
        if (!ENABLED || !follower.following()) {
            return Trip.NONE;
        }
        double now = timer.milliseconds();
        if (now < START_GRACE_MS) {
            return Trip.NONE;
        }
        Pose pose = follower.pose();
        Velocity v = follower.velocity();
        double speed = Math.hypot(v.vx, v.vy);
        double remaining = follower.remainingDistance();

        boolean slow = speed < BLOCKED_SPEED_IPS && Math.abs(v.omega) < Math.toRadians(BLOCKED_TURN_DPS)
                && remaining > MIN_REMAINING_IN;
        if (!slow) {
            slowSinceMs = -1;
        } else if (slowSinceMs < 0) {
            slowSinceMs = now;
        } else if (now - slowSinceMs >= BLOCKED_MS) {
            reason = String.format(Locale.US, "BLOCKED at (%.1f, %.1f): %.1f in/s for %.0f ms, %.1f in of the path left",
                    pose.x(), pose.y(), speed, now - slowSinceMs, remaining);
            return Trip.BLOCKED;
        }

        Pose closest = follower.closestPose();
        double off = closest == null ? 0.0 : Math.hypot(pose.x() - closest.x(), pose.y() - closest.y());
        if (off <= PUSHED_IN) {
            offSinceMs = -1;
        } else if (offSinceMs < 0) {
            offSinceMs = now;
        } else if (now - offSinceMs >= PUSHED_MS) {
            reason = String.format(Locale.US, "PUSHED at (%.1f, %.1f): %.1f in off the path", pose.x(), pose.y(), off);
            return Trip.PUSHED;
        }
        return Trip.NONE;
    }

    /**
     * Follows the path {@code plan} gives (asked again from the current pose for each retry); ends
     * at the path's end, or stopped after RETRIES + 1 trips (gaveUp()).
     */
    public Command follow(String name, Supplier<Path> plan) {
        final int[] attempt = {0};
        final boolean[] waiting = {false};
        final boolean[] finished = {false};
        final ElapsedTime wait = new ElapsedTime();
        return new CommandBuilder()
                .setStart(() -> {
                    attempt[0] = 0;
                    waiting[0] = false;
                    finished[0] = false;
                    gaveUp = false;
                    follower.follow(plan.get());
                    start();
                })
                .setExecute(() -> {
                    if (finished[0]) {
                        return;
                    }
                    if (waiting[0]) {
                        if (wait.milliseconds() >= RETRY_WAIT_MS) {
                            waiting[0] = false;
                            log.accept(String.format(Locale.US, "%s: retry %d", name, attempt[0]));
                            follower.follow(plan.get());
                            start();
                        }
                        return;
                    }
                    if (follower.atParametricEnd()) {
                        finished[0] = true;
                        return;
                    }
                    if (check() != Trip.NONE) {
                        trips++;
                        follower.manual(DrivePowers.zero());   // stop pushing
                        if (attempt[0] >= RETRIES) {
                            gaveUp = true;
                            finished[0] = true;
                            log.accept(name + ": " + reason + "; gave up after " + (attempt[0] + 1) + " tries");
                        } else {
                            attempt[0]++;
                            waiting[0] = true;
                            wait.reset();
                            log.accept(name + ": " + reason + "; waiting " + (int) RETRY_WAIT_MS + " ms");
                        }
                    }
                })
                .setDone(() -> finished[0]);
    }

    /** The last follow() gave up (the robot is stopped short of the path's end). */
    public boolean gaveUp() {
        return gaveUp;
    }

    /** Trips since the start. */
    public int trips() {
        return trips;
    }

    /** What the last trip was. */
    public String reason() {
        return reason;
    }
}
