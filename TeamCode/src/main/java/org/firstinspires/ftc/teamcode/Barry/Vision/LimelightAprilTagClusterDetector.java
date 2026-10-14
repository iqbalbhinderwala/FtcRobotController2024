package org.firstinspires.ftc.teamcode.Barry.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the center of each BIOBUZZ AprilTag cluster from Limelight tag detections.
 *
 * The Limelight reports every tag separately. Each cluster has 4 tags in a row at known
 * offsets along the sticker (Manual Figure 9-15; same layout as the SDK's
 * AprilTagGameDatabase.getBioBuzzTagLibrary()):
 * IDs 30-33 red scoring, 34-37 red audience, 38-41 blue audience, 42-45 blue scoring.
 *
 * Each visible tag i has a position p_i and a known offset u_i along the sticker, so
 * p_i = center + u_i * d, where d is the unit direction of the sticker in camera space.
 * A least-squares fit over the visible tags gives the center without using the tags'
 * (noisy) rotations. Works with any 2-4 visible tags. With a single tag the sticker is
 * assumed to face the camera (d = camera X).
 *
 * With 2+ tags it also gives the camera pose in cluster space. The cluster frame uses the
 * fitted direction d as its X axis and the average of the tags' Z axes (orthogonalized
 * against d) for the plane normal, so the noisy tag yaw is replaced by the positional fit.
 */
public class LimelightAprilTagClusterDetector {

    public static final int FIRST_ID = 30;
    private static final int TAGS_PER_CLUSTER = 4;
    private static final String[] NAMES = {"RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING"};

    private static final double INCH = 0.0254;
    /** Offset of each tag's center from the cluster center along the sticker, meters. */
    private static final double[] OFFSET_M = {-6.50 * INCH, -2.75 * INCH, 2.75 * INCH, 6.50 * INCH};

    /** Returns one detection per cluster with at least one visible tag (empty if none). */
    public List<ClusterDetection> detect(LLResult result) {
        List<ClusterDetection> detections = new ArrayList<>();
        if (result == null || !result.isValid()) {
            return detections;
        }

        // tags[cluster][slot] = {x, y, z, nx, ny, nz}: tag position in camera space (meters) and the
        // tag's Z axis in camera space, or null if not seen
        double[][][] tags = new double[NAMES.length][TAGS_PER_CLUSTER][];

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int index = tag.getFiducialId() - FIRST_ID;
            if (index < 0 || index >= NAMES.length * TAGS_PER_CLUSTER) {
                continue;
            }
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) {
                continue;
            }
            Position p = pose.getPosition().toUnit(DistanceUnit.METER);
            double[] n = zAxis(pose.getOrientation().getRoll(AngleUnit.RADIANS),
                    pose.getOrientation().getPitch(AngleUnit.RADIANS),
                    pose.getOrientation().getYaw(AngleUnit.RADIANS));
            tags[index / TAGS_PER_CLUSTER][index % TAGS_PER_CLUSTER] = new double[]{p.x, p.y, p.z, n[0], n[1], n[2]};
        }

        for (int c = 0; c < NAMES.length; c++) {
            ClusterDetection detection = estimate(c, tags[c]);
            if (detection != null) {
                detections.add(detection);
            }
        }
        return detections;
    }

    /** Convenience: the detection for one cluster index (0-3), or null if not visible. */
    public ClusterDetection find(List<ClusterDetection> detections, int firstTagId) {
        for (ClusterDetection d : detections) {
            if (d.firstTagId == firstTagId) {
                return d;
            }
        }
        return null;
    }

    private ClusterDetection estimate(int cluster, double[][] slots) {
        int n = 0;
        double meanU = 0;
        double[] meanP = new double[3];
        double[] meanN = new double[3];
        for (int s = 0; s < TAGS_PER_CLUSTER; s++) {
            if (slots[s] == null) continue;
            n++;
            meanU += OFFSET_M[s];
            for (int k = 0; k < 3; k++) {
                meanP[k] += slots[s][k];
                meanN[k] += slots[s][3 + k];
            }
        }
        if (n == 0) {
            return null;
        }
        meanU /= n;
        for (int k = 0; k < 3; k++) {
            meanP[k] /= n;
            meanN[k] /= n;
        }

        // Sticker direction: least-squares slope of position vs offset. Camera X if only 1 tag.
        double[] d = {1, 0, 0};
        if (n >= 2) {
            double suu = 0;
            double[] sup = new double[3];
            for (int s = 0; s < TAGS_PER_CLUSTER; s++) {
                if (slots[s] == null) continue;
                double du = OFFSET_M[s] - meanU;
                suu += du * du;
                for (int k = 0; k < 3; k++) sup[k] += du * (slots[s][k] - meanP[k]);
            }
            for (int k = 0; k < 3; k++) d[k] = sup[k] / suu;
        }

        double[] center = {
                meanP[0] - meanU * d[0],
                meanP[1] - meanU * d[1],
                meanP[2] - meanU * d[2]};

        double[] dir = normalize(d);
        return new ClusterDetection(NAMES[cluster], FIRST_ID + cluster * TAGS_PER_CLUSTER, n,
                center[0], center[1], center[2],
                dir[0], dir[1], dir[2],
                n >= 2 ? cameraPose(d, meanN, center) : null);
    }

    /**
     * Camera pose in the cluster frame. Cluster axes in camera space: ex = sticker direction d,
     * ez = average tag Z axis made perpendicular to ex, ey = ez x ex (right-handed, Y down).
     */
    private static ClusterDetection.CameraPose cameraPose(double[] d, double[] meanNormal, double[] center) {
        double[] ex = normalize(d);
        double along = dot(meanNormal, ex);
        double[] ez = normalize(new double[]{
                meanNormal[0] - along * ex[0],
                meanNormal[1] - along * ex[1],
                meanNormal[2] - along * ex[2]});
        double[] ey = cross(ez, ex);

        // Camera position in the cluster frame = -R^T * center, with R's columns ex, ey, ez
        double x = -dot(ex, center);
        double y = -dot(ey, center);
        double z = -dot(ez, center);

        // Camera orientation = R^T, as Rz(rz)*Ry(ry)*Rx(rx) angles
        double ry = -Math.asin(Math.max(-1, Math.min(1, ez[0])));
        double rx = Math.atan2(ez[1], ez[2]);
        double rz = Math.atan2(ey[0], ex[0]);

        return new ClusterDetection.CameraPose(x, y, z,
                Math.toDegrees(rx), Math.toDegrees(ry), Math.toDegrees(rz));
    }

    /** Z axis (third column) of Rz(yaw)*Ry(pitch)*Rx(roll), angles in radians. */
    private static double[] zAxis(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll), sr = Math.sin(roll);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        return new double[]{
                cy * sp * cr + sy * sr,
                sy * sp * cr - cy * sr,
                cp * cr};
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] normalize(double[] v) {
        double m = Math.sqrt(dot(v, v));
        return new double[]{v[0] / m, v[1] / m, v[2] / m};
    }
}
