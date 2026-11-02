package org.firstinspires.ftc.teamcode.John.Test;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.John.RobotGeometry;
import org.firstinspires.ftc.teamcode.John.ShootingZone;
import org.firstinspires.ftc.teamcode.John.Vision.HiveTags;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/**
 * The test spots of [John] Shot Calibration and how well each is covered, from every results log
 * on the robot (all earlier runs) plus the volleys saved in this run. A spot is a distance from the
 * up CELL and an angle off its axis (either CELL, either alliance: taken as the same shot), on the
 * side of the axis toward our alliance wall.
 *
 * Covered: the speeds were bracketed (a speed below and one above the best that scored at most
 * EDGE_RATE) and GOOD_BALLS balls were shot at speeds that scored GOOD_RATE (or, if none did, at the
 * best speed). Until then suggest() gives the next speed to
 * try: the table's speed first, then 50 below the lowest tried until one comes up short, then 50
 * above the highest until one goes long, then the best one again.
 *
 * Straight in front of a CELL the wall stops John at about 49"; spots the field has no room for are
 * left out.
 */
public class ShotCoverage {

    /**
     * The test spots {distance in, angle off the CELL axis deg, tier}, most useful first (numbered in
     * this order):
     *  1 = where the matches shoot from: 50-54" at 15-45 deg (the autonomous' volley 2 is about 49" at
     *      43 deg; the TeleOp's drive into range stops at about 50" at the zone's edges);
     *  2 = the long range at 45 deg, where the speed rises fastest and the window is narrowest;
     *  3 = 42" at an angle, before lowering ShootingZone.MIN_DISTANCE_IN to about 42";
     *  4 = 46" fill-ins (between spots that are already known): least new information.
     * The last four were covered on 2026-11-01 (r0273).
     */
    private static final double[][] SPOTS = {
            {50, 45, 1}, {54, 30, 1}, {54, 45, 1}, {50, 15, 1},
            {58, 45, 2}, {66, 45, 2},
            {42, 15, 3}, {42, 30, 3}, {42, 45, 3},
            {46, 0, 4}, {46, 15, 4}, {46, 30, 4}, {46, 45, 4},
            {42, 0, 0}, {48.5, 0, 0}, {50, 30, 0}, {62, 45, 0}};
    /** A volley belongs to a spot within these. */
    public static double MATCH_IN = 2.0;
    public static double MATCH_DEG = 7.5;
    public static double EDGE_RATE = 0.75;
    public static double GOOD_RATE = 0.9;
    public static int GOOD_BALLS = 12;
    public static double STEP_RPM = 50;

    public static final class Spot {
        public final int number;
        public final double distance, angleDeg;
        /** 1 most useful .. 4 least (see SPOTS); 0 = covered before this list. */
        public final int tier;
        /** {made, balls, volleys} per speed (rounded to 25 RPM). */
        final TreeMap<Integer, int[]> bySpeed = new TreeMap<>();

        Spot(int number, double distance, double angleDeg, int tier) {
            this.number = number;
            this.distance = distance;
            this.angleDeg = angleDeg;
            this.tier = tier;
        }

        public String name() {
            return String.format(Locale.US, "#%d %.0f in / %.0f deg%s", number, distance, angleDeg,
                    tier == 4 ? " (optional)" : "");
        }

        public int volleys() {
            int n = 0;
            for (int[] v : bySpeed.values()) n += v[2];
            return n;
        }

        /** "2275 4/4  2300 8/8 ..." */
        public String speeds() {
            StringBuilder sb = new StringBuilder();
            for (Map.Entry<Integer, int[]> e : bySpeed.entrySet()) {
                if (sb.length() > 0) sb.append("  ");
                sb.append(e.getKey()).append(' ').append(e.getValue()[0]).append('/').append(e.getValue()[1]);
            }
            return sb.length() == 0 ? "no volleys yet" : sb.toString();
        }
    }

    /** The next speed to try at a spot, and why; covered when nothing more is needed. */
    public static final class Suggestion {
        public final double rpm;
        public final String why;
        public final boolean covered;

        Suggestion(double rpm, String why, boolean covered) {
            this.rpm = rpm;
            this.why = why;
            this.covered = covered;
        }
    }

    private final List<Spot> spots = new ArrayList<>();
    private int loadedVolleys;
    private int loadedFiles;

    public ShotCoverage() {
        double maxAlong = 72.0 - RobotGeometry.WIDTH_IN / 2.0 - Math.abs(HiveTags.cellX(true, true));
        int n = 1;
        for (double[] s : SPOTS) {
            if (s[0] * Math.cos(Math.toRadians(s[1])) <= maxAlong + 0.5) {
                spots.add(new Spot(n++, s[0], s[1], (int) s[2]));
            }
        }
    }

    /** Reads every r*_john_shot_results_*.csv in the robot's log folder. */
    public void loadLogs() {
        File[] files = new File(AppUtil.FIRST_FOLDER, "john_logs").listFiles();
        if (files == null) return;
        Arrays.sort(files);
        for (File f : files) {
            if (f.getName().contains("_john_shot_results_") && f.getName().endsWith(".csv")) {
                loadFile(f);
            }
        }
    }

    private void loadFile(File f) {
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            String header = r.readLine();
            if (header == null) return;
            List<String> cols = Arrays.asList(header.split(","));
            int iD = cols.indexOf("distance_in"), iA = cols.indexOf("off_axis_deg"), iR = cols.indexOf("rpm_set");
            int iM = cols.indexOf("made"), iB = cols.indexOf("balls"), iS = cols.indexOf("steps_done");
            if (iD < 0 || iA < 0 || iR < 0 || iM < 0) return;
            loadedFiles++;
            String line;
            while ((line = r.readLine()) != null) {
                String[] v = line.split(",", -1);
                try {
                    if (iS >= 0 && Integer.parseInt(v[iS].trim()) == 0) continue;   // nothing fired
                    int balls = iB >= 0 ? Integer.parseInt(v[iB].trim()) : 4;
                    if (add(Double.parseDouble(v[iD]), Double.parseDouble(v[iA]), Double.parseDouble(v[iR]),
                            Integer.parseInt(v[iM].trim()), balls) != null) {
                        loadedVolleys++;
                    }
                } catch (RuntimeException ignored) {
                    // a short or bad row
                }
            }
        } catch (IOException ignored) {
            // unreadable: skip
        }
    }

    /** Adds a volley; returns its spot, or null if it is not near one. */
    public Spot add(double distance, double offAxisDeg, double rpm, int made, int balls) {
        Spot s = spotAt(distance, offAxisDeg);
        if (s != null) {
            int key = (int) (Math.round(rpm / 25.0) * 25);
            int[] v = s.bySpeed.get(key);
            if (v == null) {
                v = new int[3];
                s.bySpeed.put(key, v);
            }
            v[0] += made;
            v[1] += balls;
            v[2]++;
        }
        return s;
    }

    /** The spot this distance / angle belongs to, or null. */
    public Spot spotAt(double distance, double offAxisDeg) {
        Spot best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (Spot s : spots) {
            double dd = Math.abs(distance - s.distance), da = Math.abs(Math.abs(offAxisDeg) - s.angleDeg);
            if (dd <= MATCH_IN && da <= MATCH_DEG) {
                double score = dd / MATCH_IN + da / MATCH_DEG;
                if (score < bestScore) {
                    bestScore = score;
                    best = s;
                }
            }
        }
        return best;
    }

    public Suggestion suggest(Spot s, double tableRpm) {
        if (s.bySpeed.isEmpty()) {
            return new Suggestion(round25(tableRpm), "first volley: the table's speed", false);
        }
        int best = -1;
        double bestRate = -1;
        int bestBalls = 0;
        for (Map.Entry<Integer, int[]> e : s.bySpeed.entrySet()) {
            double rate = e.getValue()[0] / (double) Math.max(1, e.getValue()[1]);
            if (rate > bestRate + 1e-9 || (Math.abs(rate - bestRate) < 1e-9 && e.getValue()[1] > bestBalls)) {
                best = e.getKey();
                bestRate = rate;
                bestBalls = e.getValue()[1];
            }
        }
        boolean low = false, high = false;
        int goodBalls = 0;
        for (Map.Entry<Integer, int[]> e : s.bySpeed.entrySet()) {
            double rate = e.getValue()[0] / (double) Math.max(1, e.getValue()[1]);
            if (rate <= EDGE_RATE && e.getKey() < best) low = true;
            if (rate <= EDGE_RATE && e.getKey() > best) high = true;
            if (rate >= GOOD_RATE) goodBalls += e.getValue()[1];
        }
        if (goodBalls == 0) goodBalls = bestBalls;   // nothing scores 90% (a narrow window): the best speed's balls
        if (!low) {
            return new Suggestion(s.bySpeed.firstKey() - STEP_RPM, "lower, to find where it goes short", false);
        }
        if (!high) {
            return new Suggestion(s.bySpeed.lastKey() + STEP_RPM, "higher, to find where it goes long", false);
        }
        if (goodBalls < GOOD_BALLS) {
            return new Suggestion(best, String.format(Locale.US, "repeat the best speed (%d of %d balls)",
                    goodBalls, GOOD_BALLS), false);
        }
        return new Suggestion(best, "COVERED (best " + best + ")", true);
    }

    public boolean covered(Spot s) {
        return suggest(s, 0).covered;
    }

    /**
     * Where to stand for a spot at this CELL: on the side of its axis toward our alliance wall, turned
     * to shoot; X kept inside the walls (the 48.5" spot is flush on the back wall).
     */
    public static Pose pose(Spot s, double cellX, double cellY, boolean audience, boolean red) {
        double axis = audience ? 0.0 : Math.PI;
        double side = (audience ? 1.0 : -1.0) * (red ? -1.0 : 1.0);
        double a = axis + side * Math.toRadians(s.angleDeg);
        double x = cellX + s.distance * Math.cos(a);
        double y = cellY + s.distance * Math.sin(a);
        double heading = ShootingZone.shootingHeading(x, y, cellX, cellY);
        double c = Math.abs(Math.cos(heading)), sn = Math.abs(Math.sin(heading));
        double halfX = RobotGeometry.LENGTH_IN / 2.0 * c + RobotGeometry.WIDTH_IN / 2.0 * sn;
        x = Math.max(-72.0 + halfX, Math.min(72.0 - halfX, x));
        return new Pose(x, y, ShootingZone.shootingHeading(x, y, cellX, cellY));
    }

    /** The most useful uncovered spot, or the next uncovered one after {@code after} (cycling); null if all are covered. */
    public Spot nextUncovered(Spot after) {
        List<Spot> open = new ArrayList<>();
        for (Spot s : spots) {
            if (!covered(s)) open.add(s);
        }
        if (open.isEmpty()) return null;
        if (after == null) return open.get(0);
        int i = open.indexOf(after);
        return open.get((i + 1) % open.size());
    }

    public int uncoveredCount() {
        int n = 0;
        for (Spot s : spots) if (!covered(s)) n++;
        return n;
    }

    public int spotCount() {
        return spots.size();
    }

    public String loaded() {
        return String.format(Locale.US, "%d earlier volleys at test spots from %d results logs", loadedVolleys, loadedFiles);
    }

    private static double round25(double rpm) {
        return Math.round(rpm / 25.0) * 25.0;
    }
}
