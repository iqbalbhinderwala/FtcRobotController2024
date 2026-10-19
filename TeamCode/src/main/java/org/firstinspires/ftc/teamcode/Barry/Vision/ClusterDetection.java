package org.firstinspires.ftc.teamcode.Barry.Vision;

/**
 * Center of one BIOBUZZ AprilTag cluster, as seen by the Limelight.
 *
 * Position is in Limelight camera space, in meters: (0,0,0) at the lens,
 * X+ right, Y+ down, Z+ out of the camera. Compose with the fixed camera-to-robot
 * transform to get robot-relative values.
 */
public class ClusterDetection {

    /** Cluster name, e.g. "RED SCORING". */
    public final String name;
    /** First tag ID of the cluster (30, 34, 38 or 42). */
    public final int firstTagId;
    /** Number of tags the estimate is based on (1-4). 1 is less accurate: assumes the cluster faces the camera. */
    public final int tagCount;
    public final double x, y, z;
    /** Unit vector along the sticker (toward higher tag IDs) in Limelight camera space. */
    public final double dirX, dirY, dirZ;
    /**
     * Distance along the sticker between the outermost visible tags (inches): 0 with 1 tag, 3.75 for
     * two neighbors, 13 for the two outer tags. The sticker direction is only as good as this span.
     */
    public final double spanIn;
    /** Camera pose in the cluster's own frame, or null when fewer than 2 tags are visible. */
    public final CameraPose cameraPose;

    public ClusterDetection(String name, int firstTagId, int tagCount, double x, double y, double z,
                            double dirX, double dirY, double dirZ, double spanIn, CameraPose cameraPose) {
        this.name = name;
        this.firstTagId = firstTagId;
        this.tagCount = tagCount;
        this.x = x;
        this.y = y;
        this.z = z;
        this.dirX = dirX;
        this.dirY = dirY;
        this.dirZ = dirZ;
        this.spanIn = spanIn;
        this.cameraPose = cameraPose;
    }

    /**
     * Camera pose in cluster space: origin at the cluster center, axes as Limelight target space
     * (X+ right along the sticker, Y+ down, Z+ into the sticker). A camera in front of the
     * sticker has negative Z. Position in meters, rotation in degrees using the same
     * Rz(rz)*Ry(ry)*Rx(rx) convention as Limelight poses.
     *
     * Accuracy depends on how far apart the visible tags are: 3-4 tags, or the two outer
     * tags, are good; two neighboring tags give a noisy rotation.
     */
    public static class CameraPose {
        public final double x, y, z;
        public final double rxDeg, ryDeg, rzDeg;

        public CameraPose(double x, double y, double z, double rxDeg, double ryDeg, double rzDeg) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.rxDeg = rxDeg;
            this.ryDeg = ryDeg;
            this.rzDeg = rzDeg;
        }
    }

    public double rangeMeters() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    /** Horizontal angle to the center. Positive = to the right of the camera axis. */
    public double bearingDeg() {
        return Math.toDegrees(Math.atan2(x, z));
    }

    /** Vertical angle to the center. Positive = above the camera axis. */
    public double elevationDeg() {
        return Math.toDegrees(Math.atan2(-y, Math.hypot(x, z)));
    }
}
