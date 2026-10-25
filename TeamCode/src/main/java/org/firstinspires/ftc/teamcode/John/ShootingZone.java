package org.firstinspires.ftc.teamcode.John;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.John.Vision.HiveTags;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Where John can shoot from at an up CELL, FTC field coordinates (inches). Distances are from the
 * robot center to the CELL's tag cluster center, seen from above (see HiveTags).
 *
 * The zone (tested 2026-10-23):
 *  - at least MIN_DISTANCE_IN (48") from the CELL: centered on the C1 / C2 seam (about 35") was too
 *    close, and about 47" (r0157, a ball on the lower rim) too; C1 at 50.3" is fine;
 *  - at most MAX_OFF_AXIS_DEG between the line from the CELL to the robot and the way the CELL
 *    opening faces (outward along the arm: +X for an audience CELL, -X for a far CELL). The widest
 *    tested shot was from A1 at the audience wall (43.2 deg), rounded to 45; A2 (64 deg) is too
 *    angled. So an up far CELL can only be shot from the far half of the field.
 *
 * nearestSpot() finds the closest place inside the zone on our own half (MARGIN_IN / MARGIN_DEG
 * inside its edges, on the field at the shooting heading); route() goes around the HIVE frame to it,
 * on our side of it when it can.
 */
@Config("JohnShootingZone")
public class ShootingZone {

    /**
     * 48 (was 43.6, from the C1 / C2 seam test): volley 2 of r0157 from about 47" (picked at the old
     * minimum + MARGIN_IN) put a ball on the lower rim; from C1 at 50.3" all go in. nearestSpot()
     * now picks spots at 48 + MARGIN_IN = 50".
     */
    public static double MIN_DISTANCE_IN = 48.0;
    public static double MAX_OFF_AXIS_DEG = 45.0;
    /** How far inside the zone's edges nearestSpot() aims, so odometry error still lands inside. */
    public static double MARGIN_IN = 2.0;
    public static double MARGIN_DEG = 3.0;
    /** Gap kept from the walls at the shooting heading. */
    public static double WALL_MARGIN_IN = 1.0;
    /** How far beyond the minimum distance nearestSpot() looks. */
    public static double SEARCH_DEPTH_IN = 60.0;
    /**
     * The HIVE frame seen from above (both HIVES, X -20 to 20, Y -25 to 25), and the extra room the
     * robot keeps from it when driving around it.
     */
    public static double HIVE_HALF_X = 20.0;
    public static double HIVE_HALF_Y = 25.0;
    /** 4 (was 2): the autonomous clipped a HIVE leg going around it (2026-10-24). */
    public static double HIVE_CLEARANCE_IN = 4.0;

    private static final double FIELD_HALF = 72.0;

    private ShootingZone() {}

    /** Horizontal distance from the robot center to the CELL. */
    public static double distance(double x, double y, double cellX, double cellY) {
        return Math.hypot(cellX - x, cellY - y);
    }

    /** Degrees between the line from the CELL to the robot and the way the CELL opening faces. */
    public static double offAxisDeg(double x, double y, double cellX, double cellY, boolean audienceCell) {
        double axis = audienceCell ? 0.0 : Math.PI;
        return Math.abs(Math.toDegrees(normalize(Math.atan2(y - cellY, x - cellX) - axis)));
    }

    /** Why a robot position cannot shoot at the CELL ("too close ...", "too angled ..."), or null if it can. */
    public static String problem(double x, double y, double cellX, double cellY, boolean audienceCell) {
        double d = distance(x, y, cellX, cellY);
        if (d < MIN_DISTANCE_IN) {
            return String.format(Locale.US, "too close (%.1f in, min %.1f)", d, MIN_DISTANCE_IN);
        }
        double off = offAxisDeg(x, y, cellX, cellY, audienceCell);
        if (off > MAX_OFF_AXIS_DEG) {
            return String.format(Locale.US, "too angled (%.0f deg off the CELL axis, max %.0f)", off, MAX_OFF_AXIS_DEG);
        }
        return null;
    }

    /** Robot heading (radians) that points the shooter from (x, y) at the CELL. */
    public static double shootingHeading(double x, double y, double cellX, double cellY) {
        return normalize(RobotGeometry.headingToShoot(Math.atan2(cellY - y, cellX - x)));
    }

    /**
     * The closest pose to {@code from} inside the zone, MARGIN_IN / MARGIN_DEG inside its edges, on
     * our own half (red: Y <= 0, blue: Y >= 0) and on the field with the robot turned to shoot,
     * heading = shootingHeading; null if there is none.
     */
    public static Pose nearestSpot(Pose from, double cellX, double cellY, boolean audienceCell, boolean red) {
        double axis = audienceCell ? 0.0 : Math.PI;
        double maxOff = Math.max(0.0, MAX_OFF_AXIS_DEG - MARGIN_DEG);
        Pose best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (double r = MIN_DISTANCE_IN + MARGIN_IN; r <= MIN_DISTANCE_IN + MARGIN_IN + SEARCH_DEPTH_IN; r += 1.0) {
            for (double off = -maxOff; off <= maxOff + 1e-9; off += 1.0) {
                double a = axis + Math.toRadians(off);
                double x = cellX + r * Math.cos(a);
                double y = cellY + r * Math.sin(a);
                double heading = shootingHeading(x, y, cellX, cellY);
                if ((red ? y > 0 : y < 0) || !onField(x, y, heading)) {
                    continue;
                }
                double d = Math.hypot(x - from.x(), y - from.y());
                if (d < bestDistance) {
                    bestDistance = d;
                    best = new Pose(x, y, heading);
                }
            }
        }
        return best;
    }

    /** True when the robot at (x, y) with this heading is inside the walls (with WALL_MARGIN_IN). */
    public static boolean onField(double x, double y, double heading) {
        double c = Math.abs(Math.cos(heading));
        double s = Math.abs(Math.sin(heading));
        double halfX = RobotGeometry.LENGTH_IN / 2.0 * c + RobotGeometry.WIDTH_IN / 2.0 * s;
        double halfY = RobotGeometry.LENGTH_IN / 2.0 * s + RobotGeometry.WIDTH_IN / 2.0 * c;
        return Math.abs(x) <= FIELD_HALF - halfX - WALL_MARGIN_IN && Math.abs(y) <= FIELD_HALF - halfY - WALL_MARGIN_IN;
    }

    /**
     * Waypoints from {@code from} to {@code to} that keep the robot center out of the HIVE frame
     * grown by half the robot's diagonal (it may be turned any way on the way) plus
     * HIVE_CLEARANCE_IN: straight if that is clear,
     * else via one or two corners of that box (the shortest clear way), only corners on our own
     * half if that works. Includes both ends. Starting inside the grown box (right beside the
     * frame), it drives straight.
     */
    public static List<double[]> route(Pose from, Pose to, boolean red) {
        double grow = Math.hypot(RobotGeometry.LENGTH_IN, RobotGeometry.WIDTH_IN) / 2.0 + HIVE_CLEARANCE_IN;
        double hx = HIVE_HALF_X + grow;
        double hy = HIVE_HALF_Y + grow;
        double[] a = {from.x(), from.y()};
        double[] b = {to.x(), to.y()};
        if (inside(a, hx, hy) || !hits(a, b, hx, hy)) {
            return Arrays.asList(a, b);
        }
        double cx = hx + 1.0;
        double cy = hy + 1.0;
        double ours = red ? -cy : cy;
        List<double[]> own = shortestVia(a, b, new double[][]{{cx, ours}, {-cx, ours}}, hx, hy);
        if (own != null) {
            return own;
        }
        List<double[]> any = shortestVia(a, b, new double[][]{{cx, cy}, {cx, -cy}, {-cx, cy}, {-cx, -cy}}, hx, hy);
        return any != null ? any : new ArrayList<>(Arrays.asList(a, b));
    }

    /** The shortest clear way from a to b via one or two of the corners, or null. */
    private static List<double[]> shortestVia(double[] a, double[] b, double[][] corners, double hx, double hy) {
        List<double[]> best = null;
        double bestLength = Double.POSITIVE_INFINITY;
        for (double[] c1 : corners) {
            if (!hits(a, c1, hx, hy) && !hits(c1, b, hx, hy)) {
                double length = length(a, c1) + length(c1, b);
                if (length < bestLength) {
                    bestLength = length;
                    best = Arrays.asList(a, c1, b);
                }
            }
            for (double[] c2 : corners) {
                if (c2 == c1 || hits(a, c1, hx, hy) || hits(c1, c2, hx, hy) || hits(c2, b, hx, hy)) {
                    continue;
                }
                double length = length(a, c1) + length(c1, c2) + length(c2, b);
                if (length < bestLength) {
                    bestLength = length;
                    best = Arrays.asList(a, c1, c2, b);
                }
            }
        }
        return best;
    }

    private static boolean inside(double[] p, double hx, double hy) {
        return Math.abs(p[0]) < hx && Math.abs(p[1]) < hy;
    }

    /** True when the segment p-q passes through the inside of the box |x| < hx, |y| < hy (Liang-Barsky). */
    private static boolean hits(double[] p, double[] q, double hx, double hy) {
        double dx = q[0] - p[0];
        double dy = q[1] - p[1];
        double t0 = 0.0;
        double t1 = 1.0;
        double[] pv = {-dx, dx, -dy, dy};
        double[] qv = {p[0] + hx, hx - p[0], p[1] + hy, hy - p[1]};
        for (int i = 0; i < 4; i++) {
            if (Math.abs(pv[i]) < 1e-12) {
                if (qv[i] <= 0) {
                    return false;
                }
            } else {
                double t = qv[i] / pv[i];
                if (pv[i] < 0) {
                    t0 = Math.max(t0, t);
                } else {
                    t1 = Math.min(t1, t);
                }
            }
        }
        return t1 - t0 > 1e-6;
    }

    private static double length(double[] p, double[] q) {
        return Math.hypot(q[0] - p[0], q[1] - p[1]);
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
