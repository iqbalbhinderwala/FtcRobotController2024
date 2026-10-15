package org.firstinspires.ftc.teamcode.Barry.Field;

/*
 * BIOBUZZ (FTC 2026-27) HIVE AprilTag poses as a function of HIVE tilt.
 *
 * Source: Competition Manual TU04, sections 9.6 / 9.9, Figs 9-9, 9-10, 9-11, 9-15, 9-17.
 * All manual dimensions are nominal (+/- 1 in).
 *
 * Field frame (official FTC, same as Drawing.CoordinateConvention.FTC): origin at field center
 * on the tile top surface, +X toward the audience, +Y toward blue (away from the Red Wall), +Z up.
 * Inches. Pedro is this frame rotated 90 degrees with its origin at a corner: use toPedroX / toPedroY.
 *
 * Tag / cluster frame, expressed in field coordinates (right-handed):
 *   origin : tag center (or cluster center for a cluster)
 *   X+ : along the sticker toward higher tag IDs (= the viewer's right, looking at the tag)
 *   Y+ : down on the printed tag (toward the sticker's bottom edge, i.e. toward the pivot / field center)
 *   Z+ : into the sticker (away from a camera looking at it)
 * Positions are in inches; multiply by METERS_PER_INCH for Limelight-style meters.
 *
 * Tilt: each HIVE has its own tilt angle in degrees, 0 = level, stable states exactly +/-30.
 * The field is symmetric about its center (180 deg), so the sign is mirrored for blue:
 *   red  HIVE: +30 = audience (+X) CELL up, -30 = far (-X) CELL up
 *   blue HIVE: +30 = far (-X) CELL up,      -30 = audience (+X) CELL up
 * Equal tilts give a symmetric field; +30 / +30 is the match start (Fig 10-2). Under that symmetry
 * tag id on the red HIVE maps to id + 8 on the blue HIVE.
 */
public final class HiveTagGeometry {

    // ---- Numbers from the figures (inches, degrees) ----
    /** Pivot axis height above the tiles (9.6.1, Fig 9-8). */
    public static final double PIVOT_Z = 43.95;
    /** HIVE center to center (Fig 9-10); HIVES are symmetric about Y = 0. */
    public static final double HIVE_CENTER_TO_CENTER = 25.5;
    /** HIVE end to end along the arm (Fig 9-9); pivot is centered. */
    public static final double HIVE_LENGTH = 42.91;
    /** Stable tilt of the arm (Fig 9-10). */
    public static final double STABLE_TILT_DEG = 30.0;
    /** Height of the up CELL's opening bottom edge above the tiles at the stable tilt (Fig 9-10). */
    public static final double OPENING_BOTTOM_Z = 53.5;
    /** Front of CELL (opening) to the reference-hole centerline (Fig 9-15). */
    public static final double HOLES_FROM_FRONT = 9.938;
    /** Reference-hole centerline to the tag centerline (Fig 9-15). */
    public static final double TAG_ABOVE_HOLES = 2.75;
    /** Tag center offsets across the arm from the cluster center, in order of increasing field Y (Fig 9-15). */
    public static final double[] TAG_OFFSETS = {-6.5, -2.75, 2.75, 6.5};
    /** Tag edge length (9.9). */
    public static final double TAG_SIZE = 3.25;
    public static final double METERS_PER_INCH = 0.0254;

    // ---- Derived ----
    public static final double HALF_LENGTH = HIVE_LENGTH / 2.0;                              // 21.455
    /** Tag centers, distance from the pivot along the arm: 21.455 - (9.938 - 2.75) = 14.27. */
    public static final double TAG_FROM_PIVOT = HALF_LENGTH - (HOLES_FROM_FRONT - TAG_ABOVE_HOLES);
    /**
     * Pivot height above the CELL floor line (the floor carries the sticker), solved from the
     * up CELL's opening bottom: OPENING_BOTTOM_Z = PIVOT_Z + HALF_LENGTH*sin30 - h*cos30  ->  h = 1.36.
     */
    public static final double PIVOT_ABOVE_FLOOR =
            (PIVOT_Z + HALF_LENGTH * Math.sin(Math.toRadians(STABLE_TILT_DEG)) - OPENING_BOTTOM_Z)
                    / Math.cos(Math.toRadians(STABLE_TILT_DEG));

    // IDs in order of increasing field Y (Fig 9-17).
    private static final int[] RED_FAR = {33, 32, 31, 30};
    private static final int[] RED_AUDIENCE = {34, 35, 36, 37};
    private static final int[] BLUE_AUDIENCE = {38, 39, 40, 41};
    private static final int[] BLUE_FAR = {45, 44, 43, 42};

    /** Pose of one tag (or, with id = -1, a cluster center) in the field frame. */
    public static final class TagPose {
        public final int id;
        public final boolean red;       // red HIVE (Y < 0) or blue
        public final boolean audience;  // audience CELL (+X) or far CELL (-X)
        /** Tag center, field frame, inches. */
        public final double x, y, z;
        /** Tag-frame X, Y, Z axes as unit vectors in the field frame. */
        public final double[] xAxis, yAxis, zAxis;

        TagPose(int id, boolean red, boolean audience, double x, double y, double z,
                double[] xAxis, double[] yAxis, double[] zAxis) {
            this.id = id; this.red = red; this.audience = audience;
            this.x = x; this.y = y; this.z = z;
            this.xAxis = xAxis; this.yAxis = yAxis; this.zAxis = zAxis;
        }

        /**
         * Rotation from the tag frame to the field frame (columns = tag X, Y, Z axes in field
         * coordinates, row-major). Proper rotation, det +1. field = R * tag + position.
         */
        public double[][] rotationMatrix() {
            return new double[][] {
                    {xAxis[0], yAxis[0], zAxis[0]},
                    {xAxis[1], yAxis[1], zAxis[1]},
                    {xAxis[2], yAxis[2], zAxis[2]}};
        }

        @Override public String toString() {
            return String.format("id %d  pos (%.2f, %.2f, %.2f) in  X+ (%.3f, %.3f, %.3f)  Y+ (%.3f, %.3f, %.3f)  Z+ (%.3f, %.3f, %.3f)",
                    id, x, y, z, xAxis[0], xAxis[1], xAxis[2], yAxis[0], yAxis[1], yAxis[2],
                    zAxis[0], zAxis[1], zAxis[2]);
        }
    }

    private HiveTagGeometry() {}

    /** Field-frame Y (toward blue) to Pedro X (origin at the left edge, 144 in field). */
    public static double toPedroX(double fieldY) {
        return 72.0 + fieldY;
    }

    /** Field-frame X (toward the audience) to Pedro Y (audience wall at Y = 0). */
    public static double toPedroY(double fieldX) {
        return 72.0 - fieldX;
    }

    /**
     * Pose of HIVE tag {@code id} (30..45) when its HIVE is tilted {@code tiltDeg}
     * (red: +30 = audience CELL up; blue: +30 = far CELL up).
     */
    public static TagPose tagPose(int id, double tiltDeg) {
        boolean red; boolean audience; int index;
        if ((index = indexOf(RED_FAR, id)) >= 0)            { red = true;  audience = false; }
        else if ((index = indexOf(RED_AUDIENCE, id)) >= 0)  { red = true;  audience = true;  }
        else if ((index = indexOf(BLUE_AUDIENCE, id)) >= 0) { red = false; audience = true;  }
        else if ((index = indexOf(BLUE_FAR, id)) >= 0)      { red = false; audience = false; }
        else throw new IllegalArgumentException("Not a HIVE tag: " + id);
        return pose(id, red, audience, TAG_OFFSETS[index], tiltDeg);
    }

    /**
     * Pose of a cluster center (midway between the 2nd and 3rd tags, on the HIVE center line).
     * @param red      red HIVE (true) or blue
     * @param audience audience CELL (+X, true) or far CELL (-X)
     */
    public static TagPose clusterPose(boolean red, boolean audience, double tiltDeg) {
        return pose(-1, red, audience, 0.0, tiltDeg);
    }

    private static TagPose pose(int id, boolean red, boolean audience, double acrossOffset, double tiltDeg) {
        // Physical rotation about the pivot axis, +a = audience end up. Mirrored for blue (see header).
        double a = Math.toRadians(red ? tiltDeg : -tiltDeg);
        double s = audience ? 1.0 : -1.0;            // which end of the arm
        double p = s * TAG_FROM_PIVOT;                // position along the arm
        double u = 0.0;                               // on the CELL floor
        double h = PIVOT_ABOVE_FLOOR;
        double hiveY = (red ? -1.0 : 1.0) * HIVE_CENTER_TO_CENTER / 2.0;

        // Rigid rotation of the HIVE about its pivot axis (parallel to Y).
        double x = p * Math.cos(a) - (u - h) * Math.sin(a);
        double y = hiveY + acrossOffset;
        double z = PIVOT_Z + p * Math.sin(a) + (u - h) * Math.cos(a);

        // Z+: into the sticker = toward the CELL, i.e. "up" in the HIVE frame.
        double[] zAxis = {-Math.sin(a), 0.0, Math.cos(a)};
        // Top of the printed tag points outward along the arm (sticker bottom edge faces field center),
        // so Y+ (down on the tag) points inward, toward the pivot.
        double[] yAxis = {-s * Math.cos(a), 0.0, -s * Math.sin(a)};
        // Right-handed: X = Y x Z. This runs toward higher IDs (Fig 9-15/9-17).
        double[] xAxis = cross(yAxis, zAxis);
        return new TagPose(id, red, audience, x, y, z, xAxis, yAxis, zAxis);
    }

    /** All 16 HIVE tags, given each HIVE's tilt. */
    public static TagPose[] allTagPoses(double redTiltDeg, double blueTiltDeg) {
        TagPose[] out = new TagPose[16];
        for (int id = 30; id <= 45; id++) {
            out[id - 30] = tagPose(id, id <= 37 ? redTiltDeg : blueTiltDeg);
        }
        return out;
    }

    private static int indexOf(int[] ids, int id) {
        for (int i = 0; i < ids.length; i++) if (ids[i] == id) return i;
        return -1;
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }
}
