package org.firstinspires.ftc.teamcode.John.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.John.RobotGeometry;

import java.util.Locale;

/**
 * Finds whether John stands with his camera wall flush on a back wall (the audience wall or the far
 * wall), shooter facing the field, from the HIVE tags alone: no odometry, no alliance. FTC field
 * coordinates.
 *
 * Against the audience wall the heading is +90 and the robot center at X = 72 - WIDTH / 2; against
 * the far wall -90 and the mirror X. For each, the tags give the robot position for that heading
 * (TagLocalizer); the wall whose X the tags reproduce within FOUND_IN is the one, and then the pose
 * is exact: X and heading from the wall, Y from the tags. (Under the wrong heading the tags give a
 * position nowhere near that wall.)
 */
@Config("JohnWallFinder")
public class WallFinder {

    public static double FOUND_IN = 4.0;

    private final TagLocalizer audience;
    private final TagLocalizer far;

    public WallFinder(int samples) {
        audience = new TagLocalizer(samples);
        far = new TagLocalizer(samples);
    }

    public void add(HiveCamera.Frame frame) {
        audience.add(frame, Math.toRadians(90.0));
        far.add(frame, Math.toRadians(-90.0));
    }

    public void reset() {
        audience.reset();
        far.reset();
    }

    private static double wallX() {
        return 72.0 - RobotGeometry.WIDTH_IN / 2.0;
    }

    /** How far the tags' X is from the audience / far wall position (inches), or infinity. */
    public double audienceError(int minSamples) {
        return audience.count() >= minSamples ? Math.abs(audience.x() - wallX()) : Double.POSITIVE_INFINITY;
    }

    public double farError(int minSamples) {
        return far.count() >= minSamples ? Math.abs(far.x() + wallX()) : Double.POSITIVE_INFINITY;
    }

    /** The pose against the back wall the tags agree with, or null if neither (or too few tags). */
    public Pose found(int minSamples) {
        double ea = audienceError(minSamples);
        double ef = farError(minSamples);
        if (Math.min(ea, ef) > FOUND_IN) {
            return null;
        }
        boolean atAudience = ea <= ef;
        TagLocalizer fix = atAudience ? audience : far;
        Pose wall = new Pose(atAudience ? wallX() : -wallX(), 0.0, Math.toRadians(atAudience ? 90.0 : -90.0));
        return fix.alongWall(wall, minSamples, RobotGeometry.LENGTH_IN / 2.0);
    }

    public int count() {
        return Math.max(audience.count(), far.count());
    }

    /** For telemetry. */
    public String describe(int minSamples) {
        Pose p = found(minSamples);
        if (p != null) {
            return String.format(Locale.US, "against the %s wall (tags %.1f in off it, %d readings)",
                    p.x() > 0 ? "audience" : "far", Math.min(audienceError(minSamples), farError(minSamples)), count());
        }
        if (count() < minSamples) {
            return "not enough tags (" + count() + ")";
        }
        return String.format(Locale.US, "not against a back wall (tags %.1f / %.1f in off the audience / far wall)",
                audienceError(minSamples), farError(minSamples));
    }
}
