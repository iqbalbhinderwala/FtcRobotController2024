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
    /** A follow() that has not reached its end after this long gives up (it must never hang). */
    public static double FOLLOW_TIMEOUT_MS = 8000;
    /**
     * Stalled within this of the end (a follow or a creep) counts as arrived, "as far as it goes":
     * the blocked check does not look at the last MIN_REMAINING_IN, and r0186 sat 0.8" short of the
     * park approach (a ball against the wall) until the autonomous ended.
     */
    public static double ARRIVE_WITHIN_IN = 4.0;
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
        final ElapsedTime running = new ElapsedTime();
        final double[] stallNearEndMs = {-1.0};
        return new CommandBuilder()
                .setStart(() -> {
                    attempt[0] = 0;
                    waiting[0] = false;
                    finished[0] = false;
                    gaveUp = false;
                    stallNearEndMs[0] = -1.0;
                    follower.follow(plan.get());
                    running.reset();
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
                    // Stalled close to the end: as far as it goes (Pedro would never call it done)
                    Velocity vNow = follower.velocity();
                    double nowMs = timer.milliseconds();
                    if (nowMs >= START_GRACE_MS && follower.remainingDistance() <= ARRIVE_WITHIN_IN
                            && Math.hypot(vNow.vx, vNow.vy) < BLOCKED_SPEED_IPS) {
                        if (stallNearEndMs[0] < 0) {
                            stallNearEndMs[0] = nowMs;
                        } else if (nowMs - stallNearEndMs[0] >= BLOCKED_MS) {
                            follower.hold(follower.pose());
                            finished[0] = true;
                            log.accept(String.format(Locale.US, "%s: as far as it goes, %.1f in short of the end", name,
                                    follower.remainingDistance()));
                            return;
                        }
                    } else {
                        stallNearEndMs[0] = -1.0;
                    }
                    if (running.milliseconds() > FOLLOW_TIMEOUT_MS) {
                        follower.hold(follower.pose());
                        gaveUp = true;
                        finished[0] = true;
                        log.accept(String.format(Locale.US, "%s: timed out after %.1f s at (%.1f, %.1f)", name,
                                running.seconds(), follower.pose().x(), follower.pose().y()));
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

    // =========================================================================
    // Creep: slow straight drives without a Pedro path
    // =========================================================================

    /**
     * Creep (slow straight drives). A Pedro path with a speed cap (maxVelocityConstraint) is driven
     * by Pedro's coast feedforward alone, about 0.0102 power per in/s: at 12-15 in/s that is too
     * little to get John moving from a stop (r0109 / r0111: FOLLOW, 0.0 in/s, every time). Here the
     * speed is CREEP_*_IPS_PER_POWER feedforward plus the static friction push (JohnTuning KS) plus
     * CREEP_BREAKAWAY while nearly still, a pull back onto the line, and the end heading.
     */
    public static double CREEP_FORWARD_IPS_PER_POWER = 88.6;
    public static double CREEP_STRAFE_IPS_PER_POWER = 69.6;
    public static double CREEP_BREAKAWAY = 0.08;
    /** Pull back onto the line: in/s per inch off it. */
    public static double CREEP_CROSS_GAIN = 3.0;
    public static double CREEP_HEADING_P = 1.5;
    /** Done (and holding the end) this close to it. */
    public static double CREEP_END_IN = 0.75;
    /**
     * Speed feedback: in/s added to the command per in/s it is slower than asked (r0122: the
     * feedforward alone made 3 in/s of the 12 asked).
     */
    public static double CREEP_SPEED_GAIN = 2.0;
    /**
     * Stalled within MIN_REMAINING_IN of the end counts as arrived ("as far as it goes", e.g. a ball
     * pinned against the wall: r0122 stopped 1.4" short and pushed there for 15 s).
     */
    public static boolean CREEP_STALL_NEAR_END_IS_ARRIVED = true;
    /** Give up a creep after (length / speed) x this + 2 s. */
    public static double CREEP_TIMEOUT_FACTOR = 3.0;
    /** Never push along the line with less than this (until the last MIN_REMAINING_IN). */
    public static double CREEP_MIN_POWER = 0.22;

    /**
     * Drives straight from wherever the robot is to {@code end} (field frame, asked again for each
     * retry) at about {@code speedIps}, then holds it. BLOCKED / PUSHED, retries and give-up as
     * follow() (gaveUp()).
     */
    public Command creep(String name, Supplier<Pose> end, double speedIps) {
        final int[] attempt = {0};
        final boolean[] waiting = {false};
        final boolean[] finished = {false};
        final Pose[] from = {null};
        final Pose[] to = {null};
        final ElapsedTime wait = new ElapsedTime();
        final ElapsedTime running = new ElapsedTime();
        // Measured speed along the line (from the pose), and the stall timer near the end
        final double[] prevAlong = {0.0};
        final double[] prevMs = {-1.0};
        final double[] speedAlong = {0.0};
        final double[] stallNearEndMs = {-1.0};
        return new CommandBuilder()
                .setStart(() -> {
                    attempt[0] = 0;
                    waiting[0] = false;
                    finished[0] = false;
                    gaveUp = false;
                    from[0] = follower.pose();
                    to[0] = end.get();
                    prevMs[0] = -1.0;
                    speedAlong[0] = 0.0;
                    stallNearEndMs[0] = -1.0;
                    running.reset();
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
                            from[0] = follower.pose();
                            to[0] = end.get();
                            prevMs[0] = -1.0;
                            speedAlong[0] = 0.0;
                            stallNearEndMs[0] = -1.0;
                            start();
                        }
                        return;
                    }
                    Pose p = follower.pose();
                    Pose e = to[0];
                    double lx = e.x() - from[0].x();
                    double ly = e.y() - from[0].y();
                    double length = Math.hypot(lx, ly);
                    double ux = length < 1e-6 ? 1.0 : lx / length;
                    double uy = length < 1e-6 ? 0.0 : ly / length;
                    double rx = p.x() - from[0].x();
                    double ry = p.y() - from[0].y();
                    double remaining = length - (rx * ux + ry * uy);
                    double cross = rx * -uy + ry * ux;   // + = left of the line
                    if (remaining <= CREEP_END_IN) {
                        follower.hold(e);
                        finished[0] = true;
                        return;
                    }
                    // Speed along the line, from the pose
                    double nowMs = timer.milliseconds();
                    double along = length - remaining;
                    if (prevMs[0] >= 0 && nowMs > prevMs[0]) {
                        double inst = (along - prevAlong[0]) / ((nowMs - prevMs[0]) / 1000.0);
                        speedAlong[0] = 0.7 * speedAlong[0] + 0.3 * inst;
                    }
                    prevAlong[0] = along;
                    prevMs[0] = nowMs;
                    // Stalled close to the end: as far as it goes
                    if (CREEP_STALL_NEAR_END_IS_ARRIVED && remaining <= ARRIVE_WITHIN_IN && nowMs >= START_GRACE_MS
                            && Math.abs(speedAlong[0]) < BLOCKED_SPEED_IPS) {
                        if (stallNearEndMs[0] < 0) {
                            stallNearEndMs[0] = nowMs;
                        } else if (nowMs - stallNearEndMs[0] >= BLOCKED_MS) {
                            follower.hold(p);
                            finished[0] = true;
                            log.accept(String.format(Locale.US, "%s: as far as it goes, %.1f in short of the end", name,
                                    remaining));
                            return;
                        }
                    } else {
                        stallNearEndMs[0] = -1.0;
                    }
                    // Taking far too long: give up
                    double maxMs = (length / Math.max(1.0, speedIps)) * CREEP_TIMEOUT_FACTOR * 1000.0 + 2000.0;
                    if (running.milliseconds() > maxMs) {
                        follower.manual(DrivePowers.zero());
                        gaveUp = true;
                        finished[0] = true;
                        log.accept(String.format(Locale.US, "%s: timed out after %.1f s, %.1f in short", name,
                                running.seconds(), remaining));
                        return;
                    }
                    // Field velocity: along the line (faster when slower than asked), and back onto it
                    double vAlong = Math.min(speedIps, Math.max(4.0, remaining * 4.0));
                    double vCmd = Math.max(0.0, Math.min(3.0 * vAlong, vAlong + CREEP_SPEED_GAIN * (vAlong - speedAlong[0])));
                    double vx = vCmd * ux + CREEP_CROSS_GAIN * cross * uy;
                    double vy = vCmd * uy - CREEP_CROSS_GAIN * cross * ux;
                    // Robot frame
                    double h = p.heading();
                    double vf = vx * Math.cos(h) + vy * Math.sin(h);
                    double vl = -vx * Math.sin(h) + vy * Math.cos(h);
                    Velocity v = follower.velocity();
                    double kick = Math.hypot(v.vx, v.vy) < 3.0 ? CREEP_BREAKAWAY : 0.0;
                    double fwd = vf / CREEP_FORWARD_IPS_PER_POWER + push(vf, JohnTuning.FORWARD_KS + kick);
                    double left = vl / CREEP_STRAFE_IPS_PER_POWER + push(vl, JohnTuning.STRAFE_KS + kick);
                    double magnitude = Math.hypot(fwd, left);
                    if (remaining > MIN_REMAINING_IN && magnitude > 1e-6 && magnitude < CREEP_MIN_POWER) {
                        fwd *= CREEP_MIN_POWER / magnitude;
                        left *= CREEP_MIN_POWER / magnitude;
                    }
                    double turn = CREEP_HEADING_P * Math.atan2(Math.sin(e.heading() - h), Math.cos(e.heading() - h));
                    follower.manual(new DrivePowers(fwd, left, Math.max(-0.4, Math.min(0.4, turn))));

                    Trip trip = creepCheck(remaining, cross);
                    if (trip != Trip.NONE) {
                        trips++;
                        follower.manual(DrivePowers.zero());
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

    /** Static friction push in the direction of {@code velocity} (none for a tiny component). */
    private static double push(double velocity, double ks) {
        return Math.abs(velocity) < 0.5 ? 0.0 : Math.signum(velocity) * ks;
    }

    /** BLOCKED / PUSHED for a creep (its own remaining distance and offset from the line). */
    private Trip creepCheck(double remaining, double cross) {
        if (!ENABLED) {
            return Trip.NONE;
        }
        double now = timer.milliseconds();
        if (now < START_GRACE_MS) {
            return Trip.NONE;
        }
        Pose pose = follower.pose();
        Velocity v = follower.velocity();
        double speed = Math.hypot(v.vx, v.vy);
        if (speed >= BLOCKED_SPEED_IPS || remaining <= MIN_REMAINING_IN) {
            slowSinceMs = -1;
        } else if (slowSinceMs < 0) {
            slowSinceMs = now;
        } else if (now - slowSinceMs >= BLOCKED_MS) {
            reason = String.format(Locale.US, "BLOCKED at (%.1f, %.1f): %.1f in/s for %.0f ms, %.1f in left",
                    pose.x(), pose.y(), speed, now - slowSinceMs, remaining);
            return Trip.BLOCKED;
        }
        if (Math.abs(cross) <= PUSHED_IN) {
            offSinceMs = -1;
        } else if (offSinceMs < 0) {
            offSinceMs = now;
        } else if (now - offSinceMs >= PUSHED_MS) {
            reason = String.format(Locale.US, "PUSHED at (%.1f, %.1f): %.1f in off the line", pose.x(), pose.y(),
                    Math.abs(cross));
            return Trip.PUSHED;
        }
        return Trip.NONE;
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
