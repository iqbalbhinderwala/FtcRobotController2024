package org.firstinspires.ftc.teamcode.John.Vision;

import org.firstinspires.ftc.teamcode.John.RobotGeometry;
import org.firstinspires.ftc.teamcode.John.ShootingZone;

import java.util.Locale;

/**
 * A shot worked out from one camera frame and the heading only, no odometry position (for when the
 * Pinpoint fails). FTC field coordinates for the heading; distances in inches.
 *
 * The up CELL's tags are seen in the robot frame, so the cluster center is too (the tags' offsets
 * along field Y are turned into the robot frame with the heading). From that:
 *  - the turn that points the shooter at it (no position needed);
 *  - the distance to it (for the flywheel speed);
 *  - where the robot is (the CELL's known place minus that offset, as TagLocalizer does), for the
 *    shooting-zone check and the driver's hints.
 */
public final class VisionAim {

    /** Turn (radians, counter-clockwise) that points the shooter at the CELL. */
    public final double turn;
    /** Horizontal distance from the robot center to the CELL (inches). */
    public final double distance;
    /** The robot position this implies (field frame). */
    public final double robotX, robotY;
    /** The CELL (nominal place, field frame) and which one. */
    public final double cellX, cellY;
    public final boolean audience;
    public final int tags;

    private VisionAim(double turn, double distance, double robotX, double robotY, double cellX, double cellY,
                      boolean audience, int tags) {
        this.turn = turn;
        this.distance = distance;
        this.robotX = robotX;
        this.robotY = robotY;
        this.cellX = cellX;
        this.cellY = cellY;
        this.audience = audience;
        this.tags = tags;
    }

    /**
     * The shot at our HIVE's up CELL ({@code audience}) from this frame, the robot at field heading
     * {@code heading}; null if none of that CELL's tags is in the frame.
     */
    public static VisionAim solve(HiveCamera.Frame frame, boolean red, boolean audience, double heading) {
        if (frame == null) {
            return null;
        }
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double sumF = 0, sumL = 0;
        int n = 0;
        for (HiveCamera.Tag tag : frame.tags) {
            if (HiveTags.red(tag.id) != red || HiveTags.audience(tag.id) != audience) {
                continue;
            }
            // The tag's offset along field Y, in the robot frame: R(-heading) * (0, offsetY)
            double off = HiveTags.offsetY(tag.id);
            sumF += tag.forward - off * sin;
            sumL += tag.left - off * cos;
            n++;
        }
        if (n == 0) {
            return null;
        }
        double forward = sumF / n;
        double left = sumL / n;
        double bearing = Math.atan2(left, forward);   // robot frame, CCW from robot forward
        double turn = normalize(bearing - Math.toRadians(RobotGeometry.SHOOTER_YAW_DEG));
        double cellX = HiveTags.cellX(audience, true);
        double cellY = HiveTags.hiveY(red);
        // The CELL from the robot, in the field frame
        double dx = forward * cos - left * sin;
        double dy = forward * sin + left * cos;
        return new VisionAim(turn, Math.hypot(forward, left), cellX - dx, cellY - dy, cellX, cellY, audience, n);
    }

    /** Why this is not a good place to shoot from (ShootingZone), or null. */
    public String zoneProblem() {
        return ShootingZone.problem(robotX, robotY, cellX, cellY, audience);
    }

    /** A hint for the driver when out of the zone, or null. */
    public String hint() {
        double d = ShootingZone.distance(robotX, robotY, cellX, cellY);
        if (d < ShootingZone.MIN_DISTANCE_IN) {
            return String.format(Locale.US, "back up about %.0f in", ShootingZone.MIN_DISTANCE_IN - d);
        }
        if (ShootingZone.offAxisDeg(robotX, robotY, cellX, cellY, audience) > ShootingZone.MAX_OFF_AXIS_DEG) {
            return "move toward the middle (in front of the CELL)";
        }
        return null;
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
