package org.firstinspires.ftc.teamcode.John.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.John.RobotGeometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * John's Limelight 3A ("limelight" in John.xml) reading the HIVE AprilTags. Each tag is turned into
 * the robot frame (forward / left / up from the robot center on the floor, inches) with the camera
 * mount below (defaults from RobotGeometry, live-editable from the Dashboard as JohnCamera).
 *
 * Only tag positions are used, never the Limelight's robot pose (botpose): the CELLS tip, so the
 * tags have no fixed field pose (see HiveTags for what is fixed).
 *
 * Limelight pipeline PIPELINE must be an AprilTag pipeline, family 36h11, tag size 82.55 mm (3.25"),
 * or the distances are wrong.
 *
 * Limelight camera space: meters, X right, Y down, Z out of the lens.
 */
@Config("JohnCamera")
public class HiveCamera {

    public static int PIPELINE = 0;
    /** Lens position from the robot center (inches) and direction (degrees); see RobotGeometry. */
    public static double FORWARD_IN = RobotGeometry.CAMERA_FORWARD_IN;
    public static double LEFT_IN = RobotGeometry.CAMERA_LEFT_IN;
    public static double HEIGHT_IN = RobotGeometry.CAMERA_HEIGHT_IN;
    /** Direction the camera looks, counter-clockwise from robot forward (+90 = the shooter side). */
    public static double YAW_DEG = RobotGeometry.CAMERA_YAW_DEG;
    /** Upward tilt from horizontal. */
    public static double PITCH_DEG = RobotGeometry.CAMERA_PITCH_DEG;
    /** Rotation about the lens axis (clockwise as seen from behind the camera). */
    public static double ROLL_DEG = 0.0;
    /** Correction for the Limelight's distances (true = measured x scale). */
    public static double DISTANCE_SCALE = 1.0;
    /** Ignore results older than this (the Limelight stopped sending, or it was paused). */
    public static double MAX_STALENESS_MS = 200;

    private static final double INCHES_PER_METER = 39.3701;

    /** One HIVE tag seen in a frame, robot frame (inches). */
    public static final class Tag {
        public final int id;
        public final double forward, left, up;

        Tag(int id, double forward, double left, double up) {
            this.id = id;
            this.forward = forward;
            this.left = left;
            this.up = up;
        }

        /** Horizontal distance from the robot center. */
        public double distance() {
            return Math.hypot(forward, left);
        }
    }

    /** The HIVE tags of one camera frame. */
    public static final class Frame {
        public final List<Tag> tags;
        /** Capture to now, milliseconds (staleness + the Limelight's own latencies). */
        public final double ageMs;

        Frame(List<Tag> tags, double ageMs) {
            this.tags = Collections.unmodifiableList(tags);
            this.ageMs = ageMs;
        }

        public String ids() {
            StringBuilder s = new StringBuilder();
            for (Tag t : tags) {
                s.append(s.length() == 0 ? "" : " ").append(t.id);
            }
            return s.length() == 0 ? "none" : s.toString();
        }
    }

    private final Limelight3A limelight;
    private final String error;
    private long lastStampNanos = -1;
    private Frame last;

    /** Starts the Limelight. Without one in the configuration it reports that and sees nothing. */
    public HiveCamera(HardwareMap hardwareMap) {
        Limelight3A ll = null;
        String err = "";
        try {
            ll = hardwareMap.get(Limelight3A.class, "limelight");
            ll.pipelineSwitch(PIPELINE);
            ll.start();
        } catch (RuntimeException e) {
            ll = null;
            err = "no Limelight: " + e.getMessage();
        }
        limelight = ll;
        error = err;
    }

    /** This loop's new frame (only once per camera frame), or null if none / too old. */
    public Frame poll() {
        if (limelight == null) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || result.getControlHubTimeStampNanos() == lastStampNanos) {
            return null;
        }
        lastStampNanos = result.getControlHubTimeStampNanos();
        double ageMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        if (result.getStaleness() > MAX_STALENESS_MS) {
            return null;
        }
        List<Tag> tags = new ArrayList<>();
        if (result.isValid()) {
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                if (!HiveTags.isHiveTag(f.getFiducialId())) {
                    continue;
                }
                Pose3D pose = f.getTargetPoseCameraSpace();
                if (pose == null) {
                    continue;
                }
                Position p = pose.getPosition().toUnit(DistanceUnit.METER);
                if (p.z <= 0) {
                    continue;   // no pose solution
                }
                double[] r = toRobot(p.x, p.y, p.z);
                tags.add(new Tag(f.getFiducialId(), r[0], r[1], r[2]));
            }
        }
        last = new Frame(tags, ageMs);
        return last;
    }

    /** The last frame poll() returned, or null. */
    public Frame last() {
        return last;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public boolean available() {
        return limelight != null;
    }

    public String status() {
        if (limelight == null) {
            return error;
        }
        return String.format(Locale.US, "%s, last frame %s", limelight.isConnected() ? "connected" : "NOT CONNECTED",
                last == null ? "none" : String.format(Locale.US, "%.0f ms old, tags %s", last.ageMs, last.ids()));
    }

    /** Limelight camera-space point (meters) -> robot frame {forward, left, up} (inches). */
    public static double[] toRobot(double cx, double cy, double cz) {
        double scale = INCHES_PER_METER * DISTANCE_SCALE;
        // Camera-aligned axes: along the lens, left, up
        double forward = cz * scale;
        double cameraLeft = -cx * scale;
        double cameraUp = -cy * scale;

        double roll = Math.toRadians(ROLL_DEG);
        double left = cameraLeft * Math.cos(roll) - cameraUp * Math.sin(roll);
        double up = cameraLeft * Math.sin(roll) + cameraUp * Math.cos(roll);

        double pitch = Math.toRadians(PITCH_DEG);
        double levelForward = forward * Math.cos(pitch) - up * Math.sin(pitch);
        double levelUp = forward * Math.sin(pitch) + up * Math.cos(pitch);

        double yaw = Math.toRadians(YAW_DEG);
        return new double[]{
                FORWARD_IN + levelForward * Math.cos(yaw) - left * Math.sin(yaw),
                LEFT_IN + levelForward * Math.sin(yaw) + left * Math.cos(yaw),
                HEIGHT_IN + levelUp};
    }
}
