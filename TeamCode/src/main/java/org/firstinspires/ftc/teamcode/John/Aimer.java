package org.firstinspires.ftc.teamcode.John;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Vision.CellTracker;

import java.util.Locale;

/**
 * Turns John in place so the shooter points at the up CELL (CellTracker), used by the autonomous
 * and the TeleOp. start(), then update() every loop until it says AIMED or FAILED.
 *
 * It holds the current position at the aim heading. Once aligned (within AIM_TOLERANCE_DEG and
 * standing still for AIM_SETTLE_MS) the camera, which looks out of the shooter side, sees the CELL:
 * if that moves the target, it re-aims (up to MAX_REFINES times). Aimed at the nominal CELL with no
 * sighting yet, it waits up to VISION_WAIT_MS for one. After AIM_TIMEOUT_MS: AIMED if within
 * AIM_MAX_ERROR_DEG, else FAILED.
 *
 * facingTarget(): a heading function for the drive to the shooting spot, so John turns toward the
 * CELL along the way and arrives aimed (the Aimer then only refines).
 *
 * keepAim(), every loop of a volley: feed only while the shooter is within FIRE_TOLERANCE_DEG of
 * the aim from where John really is. A bump is recovered by the hold (Pedro drives back to the
 * held pose); pushed more than HOLD_SHIFT_IN (another robot leaning on him), it aims again from
 * where he is instead of fighting back.
 */
@Config("JohnAimer")
public class Aimer {

    public static double AIM_TOLERANCE_DEG = 1.5;
    public static double AIM_SETTLE_MS = 150;
    public static double AIM_TIMEOUT_MS = 2500;
    public static double AIM_MAX_ERROR_DEG = 4.0;
    public static int MAX_REFINES = 3;
    /** Added to the aim heading (counter-clockwise), if the shots pull to one side. */
    public static double AIM_TRIM_DEG = 0.0;
    public static double VISION_WAIT_MS = 400;
    /** facingTarget(): on the first leg, blend from the start heading over this distance. */
    public static double BLEND_IN = 12.0;
    /** keepAim(): feed only within this of the aim. */
    public static double FIRE_TOLERANCE_DEG = 2.0;
    /** keepAim(): pushed farther than this from the held spot, aim again from there. */
    public static double HOLD_SHIFT_IN = 2.0;

    public enum Result { AIMING, AIMED, FAILED }

    private final Follower follower;
    private final CellTracker tracker;
    private final boolean camera;
    private final ElapsedTime timer = new ElapsedTime();
    private Pose aimPose;
    private int refines;
    private double alignedSinceMs = -1;
    private String status = "";
    private String event;
    private boolean onAim = true;

    /** {@code camera}: a camera is there (worth waiting for a sighting). */
    public Aimer(Follower follower, CellTracker tracker, boolean camera) {
        this.follower = follower;
        this.tracker = tracker;
        this.camera = camera;
    }

    /** Robot heading that points the shooter from {@code from} at the target (plus the trim). */
    public double aimHeading(Pose from) {
        return aimHeading(from.x(), from.y());
    }

    /** Robot heading that points the shooter from (x, y) at the target (plus the trim). */
    public double aimHeading(double x, double y) {
        double[] t = tracker.target();
        return normalize(ShootingZone.shootingHeading(x, y, t[0], t[1]) + Math.toRadians(AIM_TRIM_DEG));
    }

    /**
     * Heading function for a path leg: at each point the shooter points at the target (live, from
     * the tracker). With {@code blendFrom} (the first leg), it turns from that heading over the
     * first BLEND_IN inches instead of snapping.
     */
    public Interpolator facingTarget(Double blendFrom) {
        return (curve, t) -> {
            Vector2D p = curve.get(t);
            double h = aimHeading(p.x(), p.y());
            if (blendFrom != null && BLEND_IN > 0) {
                double w = Math.min(1.0, Math.max(0.0, t) * curve.length() / BLEND_IN);
                h = normalize(blendFrom + w * normalize(h - blendFrom));
            }
            return h;
        };
    }

    /**
     * During a volley, every loop: true while the shooter is on aim (within FIRE_TOLERANCE_DEG of
     * the aim from where the robot is), so the caller feeds only then. Pushed more than
     * HOLD_SHIFT_IN from the held spot, it holds the new spot at the aim from there; a newer
     * camera sighting that moves the aim updates the hold too.
     */
    public boolean keepAim() {
        event = null;
        if (aimPose == null) {
            return false;
        }
        Pose p = follower.pose();
        double shift = Math.hypot(p.x() - aimPose.x(), p.y() - aimPose.y());
        double needed = aimHeading(p);
        if (shift > HOLD_SHIFT_IN) {
            aimPose = new Pose(p.x(), p.y(), needed);
            follower.hold(aimPose);
            event = String.format(Locale.US, "pushed %.1f in: aiming again from (%.1f, %.1f)", shift, p.x(), p.y());
        } else if (Math.abs(normalize(needed - aimPose.heading())) > Math.toRadians(AIM_TOLERANCE_DEG) / 2.0) {
            aimPose = aimPose.withHeading(needed);
            follower.hold(aimPose);
        }
        double error = Math.abs(normalize(needed - p.heading()));
        boolean ok = error <= Math.toRadians(FIRE_TOLERANCE_DEG);
        if (ok != onAim && event == null) {
            event = ok ? "back on aim: feeding again"
                    : String.format(Locale.US, "off aim (%.1f deg): holding fire", Math.toDegrees(error));
        }
        onAim = ok;
        status = ok ? "on aim" : String.format(Locale.US, "off aim %.1f deg, holding fire", Math.toDegrees(error));
        return ok;
    }

    /** Starts turning (holding the current position). Returns a description. */
    public String start() {
        Pose here = follower.pose();
        aimPose = new Pose(here.x(), here.y(), aimHeading(here));
        follower.hold(aimPose);
        onAim = true;
        refines = 0;
        alignedSinceMs = -1;
        timer.reset();
        status = "turning";
        return String.format(Locale.US, "aim at the %s CELL: %.1f deg (%s)", tracker.upCellName(),
                Math.toDegrees(aimPose.heading()), tracker.source());
    }

    /** Call every loop after start() (after follower.update() and the tracker's update()). */
    public Result update() {
        event = null;
        if (aimPose == null) {
            return Result.FAILED;
        }
        Pose pose = follower.pose();
        double error = Math.abs(normalize(aimPose.heading() - pose.heading()));
        boolean aligned = error <= Math.toRadians(AIM_TOLERANCE_DEG) && tracker.still();
        double now = timer.milliseconds();
        if (!aligned) {
            alignedSinceMs = -1;
        } else if (alignedSinceMs < 0) {
            alignedSinceMs = now;
        }
        if (aligned && now - alignedSinceMs >= AIM_SETTLE_MS) {
            if (camera && tracker.recentSightings() == 0 && now - alignedSinceMs < AIM_SETTLE_MS + VISION_WAIT_MS) {
                status = "aimed at the nominal CELL, waiting for the camera";
                return Result.AIMING;
            }
            double newAim = aimHeading(aimPose);
            double change = Math.abs(normalize(newAim - aimPose.heading()));
            if (change > Math.toRadians(AIM_TOLERANCE_DEG) / 2.0 && refines < MAX_REFINES) {
                refines++;
                aimPose = aimPose.withHeading(newAim);
                follower.hold(aimPose);
                alignedSinceMs = -1;
                status = "refining (" + tracker.source() + ")";
                event = String.format(Locale.US, "refine %d: aim %.1f deg (%.1f deg change)", refines,
                        Math.toDegrees(newAim), Math.toDegrees(change));
                return Result.AIMING;
            }
            status = String.format(Locale.US, "aimed (%.1f deg off, %s)", Math.toDegrees(error), tracker.source());
            event = status;
            return Result.AIMED;
        }
        if (now >= AIM_TIMEOUT_MS) {
            if (error <= Math.toRadians(AIM_MAX_ERROR_DEG)) {
                status = String.format(Locale.US, "aim timeout, %.1f deg off: good enough", Math.toDegrees(error));
                event = status;
                return Result.AIMED;
            }
            status = String.format(Locale.US, "could not aim (%.1f deg off)", Math.toDegrees(error));
            event = status;
            return Result.FAILED;
        }
        return Result.AIMING;
    }

    /** The pose being held (null before start()). */
    public Pose aimPose() {
        return aimPose;
    }

    public String status() {
        return status;
    }

    /** What happened in the last update() (refine, aimed, failed), or null. */
    public String event() {
        return event;
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
