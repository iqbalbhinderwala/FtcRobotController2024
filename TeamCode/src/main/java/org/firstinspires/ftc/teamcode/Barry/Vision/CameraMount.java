package org.firstinspires.ftc.teamcode.Barry.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

/**
 * Where the Limelight sits on Barry, and conversions from Limelight camera space to the robot and
 * the Pedro field. Live-editable from FTC Dashboard.
 *
 * Limelight camera space: meters, X right, Y down, Z out of the lens.
 * Robot frame: inches, origin at the robot center on the floor, forward / left / up.
 */
@Config
public class CameraMount {

    /** Lens position relative to the robot center (inches). */
    public static double FORWARD_IN = 0.0;
    public static double LEFT_IN = 7.5;
    // public static double HEIGHT_IN = 14.0;  // intended mount
    public static double HEIGHT_IN = 70.0 / 25.4;  // TEST: current mount, 70 mm (2.76 in)
    /** Direction the camera looks, counter-clockwise from robot forward (90 = out the left side). */
    public static double YAW_DEG = 90.0;
    /** Upward tilt of the camera from horizontal. */
    public static double PITCH_DEG = 39.0;

    private static final double METERS_TO_INCHES = 39.3701;

    private CameraMount() {}

    /** Limelight camera-space point (meters) -> robot frame {forward, left, up} (inches). */
    public static double[] toRobot(double cx, double cy, double cz) {
        // Camera-aligned axes: forward along the lens, left, up (before tilt)
        double forward = cz * METERS_TO_INCHES;
        double left = -cx * METERS_TO_INCHES;
        double up = -cy * METERS_TO_INCHES;

        // Undo the upward tilt (rotation about the camera's left axis)
        double pitch = Math.toRadians(PITCH_DEG);
        double levelForward = forward * Math.cos(pitch) - up * Math.sin(pitch);
        double levelUp = forward * Math.sin(pitch) + up * Math.cos(pitch);

        // Camera yaw relative to the robot
        double yaw = Math.toRadians(YAW_DEG);
        double robotForward = levelForward * Math.cos(yaw) - left * Math.sin(yaw);
        double robotLeft = levelForward * Math.sin(yaw) + left * Math.cos(yaw);

        return new double[]{robotForward + FORWARD_IN, robotLeft + LEFT_IN, levelUp + HEIGHT_IN};
    }

    /** Limelight camera-space direction -> Pedro field direction {x, y} (horizontal part, not normalized). */
    public static double[] directionToField(double dx, double dy, double dz, Pose robot) {
        double[] tip = toRobot(dx, dy, dz);
        double forward = tip[0] - FORWARD_IN;
        double left = tip[1] - LEFT_IN;
        double h = robot.heading();
        return new double[]{
                forward * Math.cos(h) - left * Math.sin(h),
                forward * Math.sin(h) + left * Math.cos(h)};
    }

    /** Limelight camera-space point (meters) -> Pedro field {x, y, z} (inches). */
    public static double[] toField(double cx, double cy, double cz, Pose robot) {
        double[] r = toRobot(cx, cy, cz);
        double h = robot.heading();
        return new double[]{
                robot.x() + r[0] * Math.cos(h) - r[1] * Math.sin(h),
                robot.y() + r[0] * Math.sin(h) + r[1] * Math.cos(h),
                r[2]};
    }

    /**
     * Robot heading (radians) that puts the field point (x, y) on the camera axis, seen from above,
     * when the robot turns in place about its center. The line of sight is offset from the robot
     * center by its perpendicular distance d, so heading = bearing - camera yaw - asin(d / distance).
     */
    public static double headingToLookAt(Pose robot, double x, double y) {
        double dx = x - robot.x();
        double dy = y - robot.y();
        double distance = Math.hypot(dx, dy);
        double yaw = Math.toRadians(YAW_DEG);
        double offset = -FORWARD_IN * Math.sin(yaw) + LEFT_IN * Math.cos(yaw);
        if (distance <= Math.abs(offset) + 1e-6) {
            return robot.heading();
        }
        return normalize(Math.atan2(dy, dx) - yaw - Math.asin(offset / distance));
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
