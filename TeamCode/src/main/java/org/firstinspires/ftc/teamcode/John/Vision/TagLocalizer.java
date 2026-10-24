package org.firstinspires.ftc.teamcode.John.Vision;

import com.pedropathing.math.Pose;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Robot position from the HIVE tags, given the robot heading (known at a wall start, or from the
 * Pinpoint). FTC field coordinates, inches.
 *
 * Every tag seen implies a robot position: its known field position (HiveTags; its CELL up or down
 * from the measured height) minus where it is from the robot (turned into the field by the heading).
 * Y is the reliable one: a tag's Y never changes. X depends on the CELL being up or down, which the
 * height tells; the manual dimensions are +/- 1", so X is a little less sure.
 *
 * add() collects these per-tag positions over frames (the robot standing still); x() / y() are their
 * medians, so a bad tag or frame does not pull the result.
 */
public class TagLocalizer {

    private final int capacity;
    private final ArrayDeque<double[]> samples = new ArrayDeque<>();

    /** Keeps the last {@code capacity} per-tag positions. */
    public TagLocalizer(int capacity) {
        this.capacity = Math.max(1, capacity);
    }

    /** Field position {x, y, z} of a tag seen from a robot pose. */
    public static double[] fieldPoint(HiveCamera.Tag tag, Pose robot) {
        double cos = Math.cos(robot.heading());
        double sin = Math.sin(robot.heading());
        return new double[]{
                robot.x() + tag.forward * cos - tag.left * sin,
                robot.y() + tag.forward * sin + tag.left * cos,
                tag.up};
    }

    /** Robot position {x, y} this tag implies for a robot heading (radians). */
    public static double[] robotPosition(HiveCamera.Tag tag, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double dx = tag.forward * cos - tag.left * sin;
        double dy = tag.forward * sin + tag.left * cos;
        double tagX = HiveTags.cellX(HiveTags.audience(tag.id), HiveTags.isUp(tag.up));
        return new double[]{tagX - dx, HiveTags.tagY(tag.id) - dy};
    }

    /** Adds the position every tag of the frame implies; returns how many. */
    public int add(HiveCamera.Frame frame, double heading) {
        if (frame == null) {
            return 0;
        }
        for (HiveCamera.Tag tag : frame.tags) {
            samples.addLast(robotPosition(tag, heading));
            while (samples.size() > capacity) {
                samples.removeFirst();
            }
        }
        return frame.tags.size();
    }

    public void reset() {
        samples.clear();
    }

    public int count() {
        return samples.size();
    }

    /** Median robot X, or NaN without samples. */
    public double x() {
        return median(0);
    }

    /** Median robot Y, or NaN without samples. */
    public double y() {
        return median(1);
    }

    /** Spread of Y: median distance from the median (inches), or NaN. */
    public double ySpread() {
        double m = y();
        if (Double.isNaN(m)) {
            return Double.NaN;
        }
        List<Double> d = new ArrayList<>();
        for (double[] s : samples) {
            d.add(Math.abs(s[1] - m));
        }
        return median(d);
    }

    private double median(int index) {
        List<Double> v = new ArrayList<>();
        for (double[] s : samples) {
            v.add(s[index]);
        }
        return median(v);
    }

    private static double median(List<Double> v) {
        if (v.isEmpty()) {
            return Double.NaN;
        }
        Collections.sort(v);
        int n = v.size();
        return n % 2 == 1 ? v.get(n / 2) : (v.get(n / 2 - 1) + v.get(n / 2)) / 2.0;
    }

    /** The up CELL of one HIVE as seen in a frame. */
    public static final class UpCell {
        /** Audience CELL (true) or far CELL. */
        public final boolean audience;
        /** Cluster center (from above), field frame, inches. */
        public final double x, y;
        public final int tags;

        UpCell(boolean audience, double x, double y, int tags) {
            this.audience = audience;
            this.x = x;
            this.y = y;
            this.tags = tags;
        }
    }

    /**
     * The up CELL of the red or blue HIVE in this frame, seen from a robot pose: which CELL it is
     * (from the tags' heights) and where its cluster center is, or null if none of its tags is
     * visible. Seen from the same (maybe slightly wrong) pose that is later used to aim, so a small
     * odometry error cancels out.
     */
    public static UpCell upCell(HiveCamera.Frame frame, Pose robot, boolean red) {
        if (frame == null) {
            return null;
        }
        int audienceUp = 0, farUp = 0;
        double[] sumX = new double[2], sumY = new double[2];
        for (HiveCamera.Tag tag : frame.tags) {
            if (HiveTags.red(tag.id) != red) {
                continue;
            }
            boolean audience = HiveTags.audience(tag.id);
            boolean up = HiveTags.isUp(tag.up);
            // A down tag of one CELL also means the other CELL is up
            boolean upCellIsAudience = audience == up;
            if (upCellIsAudience) audienceUp++; else farUp++;
            if (up) {
                double[] p = fieldPoint(tag, robot);
                int i = audience ? 0 : 1;
                sumX[i] += p[0];
                sumY[i] += p[1] - HiveTags.offsetY(tag.id);
            }
        }
        if (audienceUp == 0 && farUp == 0) {
            return null;
        }
        boolean audience = audienceUp >= farUp;
        int i = audience ? 0 : 1;
        int n = 0;
        for (HiveCamera.Tag tag : frame.tags) {
            if (HiveTags.red(tag.id) == red && HiveTags.audience(tag.id) == audience && HiveTags.isUp(tag.up)) {
                n++;
            }
        }
        if (n == 0) {
            // Only the down CELL is visible: the up one is at its nominal place
            return new UpCell(audience, HiveTags.cellX(audience, true), HiveTags.hiveY(red), 0);
        }
        return new UpCell(audience, sumX[i] / n, sumY[i] / n, n);
    }
}
