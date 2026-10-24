package org.firstinspires.ftc.teamcode.John.Vision;

/**
 * Where the HIVE AprilTags are, FTC field coordinates (origin at the field center on the tiles, +X
 * toward the audience, +Y toward blue, +Z up, inches). Source: Competition Manual TU04 9.6 / 9.9,
 * Figs 9-9, 9-10, 9-15, 9-17 (nominal, +/- 1"); the same numbers as Barry's HiveTagGeometry.
 *
 * Each HIVE (red at Y -12.75, blue at Y +12.75) is an arm that pivots about an axis parallel to
 * field Y, with a CELL at each end: the audience CELL (+X) and the far CELL (-X). One end is always
 * up (tilted 30 deg), the other down; a HIVE TIP swaps them. Each CELL has a sticker with 4 tags on
 * its underside, in a row across the arm (along field Y). So:
 *  - a tag's Y never changes (the pivot axis is along Y): HIVE Y + its offset in the row;
 *  - its X and height depend only on whether its CELL is up or down, and the height tells which
 *    (about 49.9" up, 35.6" down: 14" apart, easy to tell even with a few inches of error).
 * Match start: the red audience CELL and the blue far CELL are up (field symmetric about the center).
 *
 * IDs: red far 30-33, red audience 34-37, blue audience 38-41, blue far 42-45.
 */
public final class HiveTags {

    /** Pivot axis height above the tiles (Fig 9-8). */
    public static final double PIVOT_Z = 43.95;
    /** HIVE center line distance from Y = 0 (HIVE center to center 25.5", Fig 9-10). */
    public static final double HIVE_Y = 25.5 / 2.0;
    /** Stable tilt of the arm (Fig 9-10). */
    public static final double TILT_DEG = 30.0;
    /** Tag centers along the arm from the pivot: 42.91 / 2 - (9.938 - 2.75) (Figs 9-9, 9-15). */
    public static final double TAG_FROM_PIVOT = 42.91 / 2.0 - (9.938 - 2.75);
    /**
     * Pivot above the CELL floor line (the floor carries the sticker), from the up CELL's opening
     * bottom at 53.5" (Fig 9-10): 53.5 = PIVOT_Z + 21.455 sin 30 - h cos 30, so h = 1.36.
     */
    public static final double PIVOT_ABOVE_FLOOR =
            (PIVOT_Z + 42.91 / 2.0 * Math.sin(Math.toRadians(TILT_DEG)) - 53.5) / Math.cos(Math.toRadians(TILT_DEG));
    /** Tag centers across the arm from the cluster center (Fig 9-15), in order of increasing field Y. */
    private static final double[] OFFSETS = {-6.5, -2.75, 2.75, 6.5};
    /** The IDs of each cluster in order of increasing field Y (Fig 9-17). */
    private static final int[][] CLUSTERS = {
            {33, 32, 31, 30},   // red far
            {34, 35, 36, 37},   // red audience
            {38, 39, 40, 41},   // blue audience
            {45, 44, 43, 42}};  // blue far

    public static final int FIRST_ID = 30;
    public static final int LAST_ID = 45;

    private HiveTags() {}

    public static boolean isHiveTag(int id) {
        return id >= FIRST_ID && id <= LAST_ID;
    }

    /** On the red HIVE (30-37) or the blue one (38-45). */
    public static boolean red(int id) {
        return id <= 37;
    }

    /** On the audience CELL (34-41) or the far CELL. */
    public static boolean audience(int id) {
        return id >= 34 && id <= 41;
    }

    /** Offset of the tag from its cluster center along field Y. */
    public static double offsetY(int id) {
        for (int[] cluster : CLUSTERS) {
            for (int i = 0; i < cluster.length; i++) {
                if (cluster[i] == id) {
                    return OFFSETS[i];
                }
            }
        }
        throw new IllegalArgumentException("not a HIVE tag: " + id);
    }

    /** Field Y of a HIVE's center line (= of both its cluster centers). */
    public static double hiveY(boolean red) {
        return red ? -HIVE_Y : HIVE_Y;
    }

    /** Field Y of a tag (fixed: the HIVE tips about an axis along Y). */
    public static double tagY(int id) {
        return hiveY(red(id)) + offsetY(id);
    }

    /** Field X of a cluster (and of each of its tags) with its CELL up or down. */
    public static double cellX(boolean audience, boolean up) {
        double a = Math.toRadians(TILT_DEG);
        double x = TAG_FROM_PIVOT * Math.cos(a) + (up ? 1.0 : -1.0) * PIVOT_ABOVE_FLOOR * Math.sin(a);
        return audience ? x : -x;
    }

    /** Height of a cluster's tags with its CELL up or down. */
    public static double cellZ(boolean up) {
        double a = Math.toRadians(TILT_DEG);
        return PIVOT_Z + (up ? 1.0 : -1.0) * TAG_FROM_PIVOT * Math.sin(a) - PIVOT_ABOVE_FLOOR * Math.cos(a);
    }

    /** True when a tag seen at this height (inches above the tiles) is on an up CELL. */
    public static boolean isUp(double heightIn) {
        return heightIn > (cellZ(true) + cellZ(false)) / 2.0;
    }
}
