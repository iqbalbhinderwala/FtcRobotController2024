package org.firstinspires.ftc.teamcode.Barry.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Barry.Field.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.Barry.Pedro.FtcToPedro;
import org.firstinspires.ftc.teamcode.Barry.Vision.CameraMount;
import org.firstinspires.ftc.teamcode.Barry.Vision.ClusterDetection;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Works out the full robot pose (x, y, heading) and the alliance during INIT from the HIVE clusters
 * the camera sees. Shared by OpModes: call update() every INIT loop with the detected clusters.
 *
 * Every visible cluster (30-33, 34-37, 38-41, 42-45) predicts the robot pose on its own: its field
 * pose comes from HiveTagGeometry for its HIVE's tilt (see TiltMode), and the camera gives the
 * cluster center and the sticker direction in the robot frame:
 *   heading  = field angle of the sticker - robot-frame angle of the sticker
 *   position = cluster center - (cluster offset in the robot frame, turned by heading)
 * The alliance follows from where the robot is: red half (Pedro x < 72) = RED, otherwise BLUE.
 *
 * Only predictions whose visible tags span at least MIN_SPAN_IN along the sticker are used (any 3-4
 * tags, or 2 tags that are not neighbors); single tags and neighboring pairs give an unreliable
 * sticker direction. Predictions are weighted by (span / distance)^2, which is how the heading
 * precision scales.
 *
 * The robot does not move during INIT, so the last WINDOW frames are averaged. Robots start against a
 * wall: with SNAP_TO_WALL, an averaged heading within WALL_HEADING_TOLERANCE_DEG of wall-parallel is
 * set exactly parallel, the position is recomputed from every prediction with that heading, and a
 * position within WALL_POSITION_TOLERANCE_IN of flush against a wall is set flush. This removes most
 * of the camera's heading error (a few degrees, which also moves the position) in the normal start.
 */
@Config
public class InitLocalizer {

    public enum TiltMode { SNAP, START, FROM_HEIGHT, MANUAL }

    /** Frames averaged for the INIT estimate. */
    public static int WINDOW = 25;
    /** Smallest span of visible tags along the sticker (inches) for a usable prediction. */
    public static double MIN_SPAN_IN = 9.0;

    /**
     * How the HIVE tilt behind each cluster is known (alliance convention for all tilts):
     * SNAP = the stable state (+/-30) whose height is closer to the measured one (default; robust to
     * a few inches of height error), START = START_TILT_DEG, FROM_HEIGHT = solved continuously from the
     * measured height, MANUAL = RED_TILT_DEG / BLUE_TILT_DEG.
     */
    public static TiltMode TILT_MODE = TiltMode.SNAP;
    /** HIVE tilt at the start, alliance convention (+30 = red audience / blue far CELL up). */
    public static double START_TILT_DEG = 30.0;
    /** MANUAL tilts. */
    public static double RED_TILT_DEG = 30.0;
    public static double BLUE_TILT_DEG = 30.0;

    /** Snap the averaged pose to the wall the robot starts against. */
    public static boolean SNAP_TO_WALL = true;
    public static double WALL_HEADING_TOLERANCE_DEG = 10.0;
    public static double WALL_POSITION_TOLERANCE_IN = 2.5;

    private static final double FIELD_SIZE = 144.0;

    /** The pose predicted by one cluster. */
    public static final class ClusterPrediction {
        public final int firstTagId;
        public final int tags;
        /** Distance between the outermost visible tags along the sticker (inches). */
        public final double spanIn;
        public final double rangeIn;
        /** Cluster center height above the floor as measured by the camera (inches). */
        public final double heightIn;
        /** HIVE tilt used for this cluster (alliance convention). */
        public final double tiltDeg;
        public final Pose pose;
        /** Weight in the fusion: (span / range)^2. */
        public final double weight;
        // For recomputing the position with another heading
        private final double fieldX, fieldY, robotForward, robotLeft;

        ClusterPrediction(int firstTagId, int tags, double spanIn, double rangeIn, double heightIn,
                          double tiltDeg, Pose pose, double fieldX, double fieldY,
                          double robotForward, double robotLeft) {
            this.firstTagId = firstTagId;
            this.tags = tags;
            this.spanIn = spanIn;
            this.rangeIn = rangeIn;
            this.heightIn = heightIn;
            this.tiltDeg = tiltDeg;
            this.pose = pose;
            this.weight = (spanIn / Math.max(1.0, rangeIn)) * (spanIn / Math.max(1.0, rangeIn));
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.robotForward = robotForward;
            this.robotLeft = robotLeft;
        }

        /** Robot position this cluster implies if the robot heading is {@code heading}. */
        public double[] positionFor(double heading) {
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            return new double[]{
                    fieldX - (robotForward * cos - robotLeft * sin),
                    fieldY - (robotForward * sin + robotLeft * cos)};
        }
    }

    /** One frame's combined estimate. */
    public static final class Estimate {
        public final HiveAuto.Alliance alliance;
        /** Pedro pose, inches / radians. */
        public final Pose pose;
        /** The clusters used, each with its own prediction. */
        public final List<ClusterPrediction> used;

        Estimate(Pose pose, List<ClusterPrediction> used) {
            this.alliance = allianceAt(pose);
            this.pose = pose;
            this.used = Collections.unmodifiableList(used);
        }

        public int tags() {
            int n = 0;
            for (ClusterPrediction p : used) n += p.tags;
            return n;
        }
    }

    private final ArrayDeque<Estimate> window = new ArrayDeque<>();
    private Estimate latest = null;

    /** The pose one visible cluster predicts (TILT_MODE), or null if not a HIVE cluster. */
    public static ClusterPrediction predict(ClusterDetection c) {
        return predict(c, TILT_MODE);
    }

    /** The pose one visible cluster predicts, or null if it is not one of the four HIVE clusters. */
    public static ClusterPrediction predict(ClusterDetection c, TiltMode mode) {
        boolean red = c.firstTagId == 30 || c.firstTagId == 34;
        boolean audience = c.firstTagId == 34 || c.firstTagId == 38;
        if (!red && c.firstTagId != 38 && c.firstTagId != 42) {
            return null;
        }

        // The cluster seen from the robot (its height above the floor needs no robot pose)
        double[] robotCenter = CameraMount.toRobot(c.x, c.y, c.z);
        double[] robotDir = CameraMount.directionToRobot(c.dirX, c.dirY, c.dirZ);

        // Its HIVE's tilt, then its known field pose at that tilt
        double tilt;
        switch (mode) {
            case SNAP:
                tilt = snapTilt(red, audience, robotCenter[2]);
                break;
            case FROM_HEIGHT:
                tilt = tiltFromHeight(red, audience, robotCenter[2]);
                break;
            case MANUAL:
                tilt = red ? RED_TILT_DEG : BLUE_TILT_DEG;
                break;
            default:
                tilt = START_TILT_DEG;
        }
        HiveTagGeometry.TagPose nominal = HiveTagGeometry.clusterPose(red, audience, tilt);
        double[] center = FtcToPedro.point(nominal.x, nominal.y);
        double[] fieldDir = FtcToPedro.direction(nominal.xAxis[0], nominal.xAxis[1]);

        double heading = CameraMount.normalize(
                Math.atan2(fieldDir[1], fieldDir[0]) - Math.atan2(robotDir[1], robotDir[0]));
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double x = center[0] - (robotCenter[0] * cos - robotCenter[1] * sin);
        double y = center[1] - (robotCenter[0] * sin + robotCenter[1] * cos);
        return new ClusterPrediction(c.firstTagId, c.tagCount, c.spanIn, c.rangeMeters() * 39.3701,
                robotCenter[2], tilt, new Pose(x, y, heading), center[0], center[1],
                robotCenter[0], robotCenter[1]);
    }

    /** The stable tilt (+/-30, alliance convention) whose cluster height is closer to the measured one. */
    public static double snapTilt(boolean red, boolean audience, double heightIn) {
        double up = HiveTagGeometry.STABLE_TILT_DEG;
        double zPlus = HiveTagGeometry.clusterPose(red, audience, up).z;
        double zMinus = HiveTagGeometry.clusterPose(red, audience, -up).z;
        return Math.abs(heightIn - zPlus) <= Math.abs(heightIn - zMinus) ? up : -up;
    }

    /**
     * HIVE tilt (alliance convention) that puts this cluster's center at the measured height. The
     * height changes steadily with tilt between the stable states, so a bisection finds it; heights
     * outside the range give the nearest stable tilt.
     */
    public static double tiltFromHeight(boolean red, boolean audience, double heightIn) {
        double lo = -HiveTagGeometry.STABLE_TILT_DEG;
        double hi = HiveTagGeometry.STABLE_TILT_DEG;
        double zLo = HiveTagGeometry.clusterPose(red, audience, lo).z;
        double zHi = HiveTagGeometry.clusterPose(red, audience, hi).z;
        boolean rising = zHi > zLo;
        if (heightIn <= Math.min(zLo, zHi)) return rising ? lo : hi;
        if (heightIn >= Math.max(zLo, zHi)) return rising ? hi : lo;
        for (int i = 0; i < 30; i++) {
            double mid = (lo + hi) / 2.0;
            double z = HiveTagGeometry.clusterPose(red, audience, mid).z;
            if ((z < heightIn) == rising) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return (lo + hi) / 2.0;
    }

    /** Combined estimate from all HIVE clusters in one frame (TILT_MODE), or null if none is usable. */
    public static Estimate fromFrame(List<ClusterDetection> clusters) {
        return fromFrame(clusters, TILT_MODE);
    }

    /** Combined estimate from all usable HIVE clusters in one frame, or null if none is usable. */
    public static Estimate fromFrame(List<ClusterDetection> clusters, TiltMode mode) {
        List<ClusterPrediction> used = new ArrayList<>();
        for (ClusterDetection c : clusters) {
            if (c.spanIn < MIN_SPAN_IN) continue;
            ClusterPrediction p = predict(c, mode);
            if (p != null) used.add(p);
        }
        if (used.isEmpty()) {
            return null;
        }
        double sw = 0, sx = 0, sy = 0, ss = 0, sc = 0;
        for (ClusterPrediction p : used) {
            sw += p.weight;
            sx += p.weight * p.pose.x();
            sy += p.weight * p.pose.y();
            ss += p.weight * Math.sin(p.pose.heading());
            sc += p.weight * Math.cos(p.pose.heading());
        }
        return new Estimate(new Pose(sx / sw, sy / sw, Math.atan2(ss, sc)), used);
    }

    /** Feed the clusters detected this loop (INIT, robot still), using TILT_MODE. */
    public void update(List<ClusterDetection> clusters) {
        update(clusters, TILT_MODE);
    }

    /** Feed the clusters detected this loop (INIT, robot still). */
    public void update(List<ClusterDetection> clusters, TiltMode mode) {
        latest = fromFrame(clusters, mode);
        if (latest == null) {
            return;
        }
        if (!window.isEmpty() && window.peekLast().alliance != latest.alliance) {
            window.clear();
        }
        window.addLast(latest);
        while (window.size() > Math.max(1, WINDOW)) {
            window.removeFirst();
        }
    }

    public void reset() {
        window.clear();
        latest = null;
    }

    /** True once a usable HIVE cluster has been seen. */
    public boolean hasFix() {
        return !window.isEmpty();
    }

    /** This loop's frame estimate, or null if no usable HIVE cluster is in view now. */
    public Estimate latest() {
        return latest;
    }

    /** Alliance from the final pose: red half (Pedro x < 72) = RED. */
    public HiveAuto.Alliance alliance() {
        Pose p = pose();
        return p == null ? null : allianceAt(p);
    }

    /** Frames in the average. */
    public int samples() {
        return window.size();
    }

    /** True when the current pose() was snapped to a wall (exact heading, maybe flush position). */
    public boolean snappedToWall() {
        Pose raw = rawAverage();
        return raw != null && SNAP_TO_WALL && Math.abs(CameraMount.normalize(raw.heading() - wallHeading(raw.heading())))
                <= Math.toRadians(WALL_HEADING_TOLERANCE_DEG);
    }

    /**
     * Averaged Pedro pose, or null without a fix. When SNAP_TO_WALL applies: heading exactly
     * wall-parallel, position recomputed from all predictions with that heading, and set flush
     * against a wall if within WALL_POSITION_TOLERANCE_IN.
     */
    public Pose pose() {
        Pose raw = rawAverage();
        if (raw == null || !snappedToWall()) {
            return raw;
        }
        double heading = wallHeading(raw.heading());
        double sw = 0, sx = 0, sy = 0;
        for (Estimate e : window) {
            for (ClusterPrediction p : e.used) {
                double[] xy = p.positionFor(heading);
                sw += p.weight;
                sx += p.weight * xy[0];
                sy += p.weight * xy[1];
            }
        }
        double x = sx / sw;
        double y = sy / sw;

        // Flush against a wall: the robot's half extent across that wall
        double cos = Math.abs(Math.cos(heading));
        double sin = Math.abs(Math.sin(heading));
        double halfX = cos * HiveAuto.ROBOT_HALF_LENGTH + sin * HiveAuto.ROBOT_HALF_WIDTH;
        double halfY = sin * HiveAuto.ROBOT_HALF_LENGTH + cos * HiveAuto.ROBOT_HALF_WIDTH;
        x = snapToWall(x, halfX);
        y = snapToWall(y, halfY);
        return new Pose(x, y, heading);
    }

    /** Standard deviation of the per-frame position around the raw average (inches). */
    public double positionSpreadIn() {
        Pose mean = rawAverage();
        if (mean == null) {
            return Double.NaN;
        }
        double sum = 0;
        for (Estimate e : window) {
            double dx = e.pose.x() - mean.x();
            double dy = e.pose.y() - mean.y();
            sum += dx * dx + dy * dy;
        }
        return Math.sqrt(sum / window.size());
    }

    /** Standard deviation of the per-frame heading around the raw average (degrees). */
    public double headingSpreadDeg() {
        Pose mean = rawAverage();
        if (mean == null) {
            return Double.NaN;
        }
        double sum = 0;
        for (Estimate e : window) {
            double d = CameraMount.normalize(e.pose.heading() - mean.heading());
            sum += d * d;
        }
        return Math.toDegrees(Math.sqrt(sum / window.size()));
    }

    /** Average of the frame estimates, before any wall snapping. */
    public Pose rawAverage() {
        if (window.isEmpty()) {
            return null;
        }
        double sx = 0, sy = 0, ss = 0, sc = 0;
        for (Estimate e : window) {
            sx += e.pose.x();
            sy += e.pose.y();
            ss += Math.sin(e.pose.heading());
            sc += Math.cos(e.pose.heading());
        }
        return new Pose(sx / window.size(), sy / window.size(), Math.atan2(ss, sc));
    }

    private static HiveAuto.Alliance allianceAt(Pose pose) {
        return pose.x() < FIELD_SIZE / 2.0 ? HiveAuto.Alliance.RED : HiveAuto.Alliance.BLUE;
    }

    /** The nearest wall-parallel heading (a multiple of 90 degrees). */
    private static double wallHeading(double heading) {
        return CameraMount.normalize(Math.round(heading / (Math.PI / 2.0)) * (Math.PI / 2.0));
    }

    /** Puts a coordinate flush against the near wall (half extent from it) when it is close enough. */
    private static double snapToWall(double value, double halfExtent) {
        if (Math.abs(value - halfExtent) <= WALL_POSITION_TOLERANCE_IN) return halfExtent;
        if (Math.abs(value - (FIELD_SIZE - halfExtent)) <= WALL_POSITION_TOLERANCE_IN) return FIELD_SIZE - halfExtent;
        return value;
    }
}
