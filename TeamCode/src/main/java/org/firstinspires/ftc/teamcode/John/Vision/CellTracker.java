package org.firstinspires.ftc.teamcode.John.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;

/**
 * Our HIVE's up CELL: which one it is (audience or far) and where it is, from the camera. FTC field
 * coordinates.
 *
 * Only frames taken while the robot stands still are used (slower than STILL_SPEED_IPS /
 * STILL_TURN_DPS, for longer than the frame's age), so the pose the camera saw from is right.
 *  - Which CELL is up: from the tag heights (TagLocalizer.upCell); it changes after UP_CELL_FRAMES
 *    frames in a row agree. At the match start: the red audience / blue far CELL.
 *  - Where: the up CELL's tag cluster center as seen from the robot's odometry pose, averaged over
 *    the sightings of the last VISION_MAX_AGE_MS; seen from the same pose that is used to aim, a
 *    small odometry drift cancels out. Without a recent sighting: the nominal place (HiveTags).
 *  - Confirmed: the last time a frame agreed with the current up CELL (msSinceConfirmed()).
 *  - Tipping: the up CELL's tags measured TIP_DROP_IN lower than a baseline (its height when a
 *    volley started) in TIP_FRAMES frames in a row: the HIVE has started to tip (tipping()).
 *
 * Seeing the tags at all: all 4 tags of a HIVE face the same way, down and toward the end that is
 * up. From the side of the up end they are easy to read; from the side of the down end they are
 * edge-on or facing away. Beside the HIVE, between its CELLS, they can be read either way.
 */
@Config("JohnCellTracker")
public class CellTracker {

    public static double STILL_SPEED_IPS = 2.0;
    public static double STILL_TURN_DPS = 10.0;
    public static double VISION_MAX_AGE_MS = 1500;
    public static int UP_CELL_FRAMES = 3;
    public static double TIP_DROP_IN = 3.0;
    public static int TIP_FRAMES = 2;
    /** An up CELL height older than this is not used. */
    public static double HEIGHT_MAX_AGE_MS = 300;

    private final boolean red;
    private boolean upAudience;
    private int disagreeFrames;
    private final ArrayDeque<double[]> sightings = new ArrayDeque<>();   // {time ms, x, y}
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime stillFor = new ElapsedTime();
    private boolean still;
    private double lastSeenMs = Double.NEGATIVE_INFINITY;
    private double lastConfirmedMs = Double.NEGATIVE_INFINITY;
    private double upHeight = Double.NaN;
    private double upHeightMs = Double.NEGATIVE_INFINITY;
    private int lowFrames;
    private double lastCountedMs = Double.NEGATIVE_INFINITY;
    private String event;

    /** For our HIVE (red or blue), starting with the match-start up CELL. */
    public CellTracker(boolean red) {
        this.red = red;
        this.upAudience = red;
    }

    /**
     * Call every loop with this loop's camera frame (or null). Returns true when the frame was
     * taken standing still (and so was used).
     */
    public boolean update(HiveCamera.Frame frame, Follower follower) {
        event = null;
        Velocity v = follower.velocity();
        boolean nowStill = Math.hypot(v.vx, v.vy) < STILL_SPEED_IPS && Math.abs(v.omega) < Math.toRadians(STILL_TURN_DPS);
        if (nowStill && !still) {
            stillFor.reset();
        }
        still = nowStill;
        if (frame == null || !still || stillFor.milliseconds() <= frame.ageMs + 30) {
            return false;
        }
        measureUpHeight(frame);
        TagLocalizer.UpCell cell = TagLocalizer.upCell(frame, follower.pose(), red);
        if (cell == null) {
            return true;
        }
        lastSeenMs = clock.milliseconds();
        if (cell.audience != upAudience) {
            if (++disagreeFrames < UP_CELL_FRAMES) {
                return true;
            }
            upAudience = cell.audience;
            sightings.clear();
            disagreeFrames = 0;
            event = "the camera sees the " + upCellName() + " CELL up";
        } else {
            disagreeFrames = 0;
        }
        lastConfirmedMs = clock.milliseconds();
        if (cell.tags > 0) {
            sightings.addLast(new double[]{clock.milliseconds(), cell.x, cell.y});
            while (sightings.size() > 20) {
                sightings.removeFirst();
            }
        }
        return true;
    }

    /** Mean height of the current up CELL's tags in this frame (whatever their height). */
    private void measureUpHeight(HiveCamera.Frame frame) {
        double sum = 0;
        int n = 0;
        for (HiveCamera.Tag tag : frame.tags) {
            if (HiveTags.red(tag.id) == red && HiveTags.audience(tag.id) == upAudience) {
                sum += tag.up;
                n++;
            }
        }
        if (n > 0) {
            upHeight = sum / n;
            upHeightMs = clock.milliseconds();
        }
    }

    /** The up CELL's measured tag height if fresh (HEIGHT_MAX_AGE_MS), else NaN. */
    public double upHeight() {
        return clock.milliseconds() - upHeightMs <= HEIGHT_MAX_AGE_MS ? upHeight : Double.NaN;
    }

    /**
     * True once the up CELL has measured TIP_DROP_IN below {@code baseline} (an earlier upHeight())
     * in TIP_FRAMES camera frames in a row (each frame counted once, however often this is called):
     * the HIVE is tipping. False without a baseline; tipping(NaN) clears the count.
     */
    public boolean tipping(double baseline) {
        double h = upHeight();
        if (Double.isNaN(baseline)) {
            lowFrames = 0;
            return false;
        }
        if (!Double.isNaN(h) && upHeightMs > lastCountedMs) {
            lastCountedMs = upHeightMs;
            lowFrames = h < baseline - TIP_DROP_IN ? lowFrames + 1 : 0;
        }
        return lowFrames >= TIP_FRAMES;
    }

    /** Low frames in a row so far (tipping()). */
    public int lowFrames() {
        return lowFrames;
    }

    /** Milliseconds since a frame agreed with the current up CELL (infinite if never). */
    public double msSinceConfirmed() {
        return clock.milliseconds() - lastConfirmedMs;
    }

    /** Target {x, y}: the mean of the recent sightings, or the up CELL's nominal place. */
    public double[] target() {
        double now = clock.milliseconds();
        double sx = 0, sy = 0;
        int n = 0;
        for (double[] s : sightings) {
            if (now - s[0] <= VISION_MAX_AGE_MS) {
                sx += s[1];
                sy += s[2];
                n++;
            }
        }
        if (n > 0) {
            return new double[]{sx / n, sy / n};
        }
        return new double[]{HiveTags.cellX(upAudience, true), HiveTags.hiveY(red)};
    }

    public int recentSightings() {
        double now = clock.milliseconds();
        int n = 0;
        for (double[] s : sightings) {
            if (now - s[0] <= VISION_MAX_AGE_MS) n++;
        }
        return n;
    }

    public String source() {
        int n = recentSightings();
        return n > 0 ? "camera, " + n + " frames" : "nominal";
    }

    /** The up CELL is the audience one (else the far one). */
    public boolean upAudience() {
        return upAudience;
    }

    /** Sets the up CELL by hand (or from an assumption); the camera still corrects it. */
    public void setUpAudience(boolean audience) {
        if (audience != upAudience) {
            sightings.clear();
            upHeight = Double.NaN;
            lastConfirmedMs = Double.NEGATIVE_INFINITY;
        }
        upAudience = audience;
        disagreeFrames = 0;
        lowFrames = 0;
    }

    public String upCellName() {
        return upAudience ? "audience" : "far";
    }

    public boolean red() {
        return red;
    }

    /** The robot stands still (as of the last update). */
    public boolean still() {
        return still;
    }

    /** Milliseconds since a still frame last showed any tag of our HIVE (infinite if never). */
    public double msSinceSeen() {
        return clock.milliseconds() - lastSeenMs;
    }

    /** What changed in the last update() (the up CELL), or null. */
    public String event() {
        return event;
    }
}
