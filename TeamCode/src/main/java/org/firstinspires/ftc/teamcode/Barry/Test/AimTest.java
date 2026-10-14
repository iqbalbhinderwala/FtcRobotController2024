package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.Barry.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Barry.Vision.ClusterDetection;
import org.firstinspires.ftc.teamcode.Barry.Vision.LimelightAprilTagClusterDetector;

import java.util.List;

/**
 * Drives the robot field-centric (Pedro ManualDrive.fieldCentric) and draws the AprilTag clusters seen by the Limelight
 * on the FTC Dashboard field, in the Pedro frame (inches).
 *
 * Camera space (Limelight): X right, Y down, Z forward. It is converted to the robot frame
 * (forward, left) using the camera mount below, then to the field using the Pedro pose.
 *
 * Setup: robot starts at (START_X, START_Y) facing START_HEADING_DEG. Camera mounted on the
 * left, CAMERA_LEFT_IN from the robot center, centered front-to-back, looking out to the left (90 deg).
 * Mount values are live-editable from FTC Dashboard.
 *
 * Controls (gamepad1): left stick drives field-centric, right stick rotates,
 * A turns the robot in place (follower.hold) so the camera looks at the center of the cluster nearest
 * the camera axis; press again to refine. Any stick input cancels.
 */
@Config
@TeleOp(name = "Aim Test", group = "BarryTest")
public class AimTest extends LinearOpMode {

    // Starting pose (Pedro frame, inches / degrees)
    public static double START_X = 12.0;
    public static double START_Y = 36.0;
    public static double START_HEADING_DEG = 0.0;

    // Camera mount relative to the robot center (inches). Positive left = toward robot's left.
    public static double CAMERA_FORWARD_IN = 0.0;
    public static double CAMERA_LEFT_IN = 7.5;
    // Camera yaw relative to robot forward, degrees, counter-clockwise positive (90 = looking out the left side)
    public static double CAMERA_YAW_DEG = 90.0;

    // Press A: turn in place (follower.hold) so the camera looks at the cluster center.
    // Heading within this many degrees of the target counts as aligned.
    public static double AIM_TOLERANCE_DEG = 2.0;

    // Pedro field-centric heading offset, degrees, added to the robot heading. For a driver facing
    // direction d in the Pedro frame (0 = +X, 90 = +Y, 180 = -X) use -d.
    public static double FIELD_CENTRIC_OFFSET_DEG = -90.0;

    // Stick scaling: 1.0 = full speed. Lower for slower, easier driving. Live-editable from Dashboard.
    public static double DRIVE_SCALE = 0.6;
    public static double TURN_SCALE = 0.5;

    private static final double STICK_DEADBAND = 0.05;
    private static final double METERS_TO_INCHES = 39.3701;

    private final PoseFactory p = PoseFactory.degrees();

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dual telemetry for Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(p.of(START_X, START_Y, START_HEADING_DEG));

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightAprilTagClusterDetector detector = new LimelightAprilTagClusterDetector();
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean lastA = false;
        String aimTarget = null;        // name of the cluster being aimed at, null when not aiming
        double aimHeading = 0.0;        // target robot heading, radians
        String aimStatus = "idle";

        while (opModeIsActive()) {
            Pose robot = follower.pose();

            LLResult result = limelight.getLatestResult();
            List<ClusterDetection> clusters = detector.detect(result);

            // Pedro conventions: forward is ahead, strafe is left, turn is counter-clockwise
            double forward = deadband(-gamepad1.left_stick_y);
            double strafe = deadband(-gamepad1.left_stick_x);
            double turn = deadband(-gamepad1.right_stick_x);
            boolean hasManualInput = (forward != 0.0 || strafe != 0.0 || turn != 0.0);

            // A: hold the current position and turn to the heading that points the camera at the center
            // of the cluster closest to the camera axis. Press again to refine; sticks cancel.
            boolean a = gamepad1.a;
            if (a && !lastA) {
                ClusterDetection nearest = closestToCameraAxis(clusters);
                if (nearest == null) {
                    aimStatus = "no cluster in view";
                } else {
                    double[] center = cameraToField(nearest.x, nearest.y, nearest.z, true, robot);
                    aimTarget = nearest.name;
                    aimHeading = headingToLookAt(robot, center[0], center[1]);
                    follower.hold(new Pose(robot.x(), robot.y(), aimHeading));
                }
            }
            lastA = a;
            if (hasManualInput && aimTarget != null) {
                aimTarget = null;
                aimStatus = "cancelled";
            }
            if (aimTarget != null) {
                double error = Math.toDegrees(normalizeAngle(aimHeading - robot.heading()));
                aimStatus = String.format("%s %s: target %.1f deg, error %.1f deg",
                        Math.abs(error) <= AIM_TOLERANCE_DEG ? "aligned with" : "turning to",
                        aimTarget, Math.toDegrees(aimHeading), error);
            }

            // Don't override a hold (A button) until the driver touches a stick
            if (hasManualInput || follower.mode() == Follower.Mode.MANUAL) {
                DrivePowers powers = ManualDrive.fieldCentric(
                        forward * DRIVE_SCALE, strafe * DRIVE_SCALE, turn * TURN_SCALE,
                        robot.heading(), Math.toRadians(FIELD_CENTRIC_OFFSET_DEG));
                follower.manual(powers);
            }
            follower.update();

            // Draw field + robot, then the detected clusters on top
            TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
            Drawing.drawRobot(packet.fieldOverlay(), robot);

            telemetry.addData("Robot", "x:%.1f y:%.1f heading:%.1f deg",
                    robot.x(), robot.y(), Math.toDegrees(robot.heading()));
            telemetry.addData("Aim (A)", aimStatus);
            telemetry.addData("Clusters in view", clusters.size());

            for (ClusterDetection c : clusters) {
                double[] center = cameraToField(c.x, c.y, c.z, true, robot);
                double[] dir = cameraToField(c.dirX, c.dirY, c.dirZ, false, robot);
                double dirLength = Math.hypot(dir[0], dir[1]);
                if (dirLength > 1e-6) {
                    dir[0] /= dirLength;
                    dir[1] /= dirLength;
                }

                String color = c.name.startsWith("RED") ? "red" : "blue";
                Drawing.drawCluster(packet.fieldOverlay(), center[0], center[1], dir[0], dir[1],
                        color, c.name + " (" + c.tagCount + ")");

                telemetry.addData(c.name, "field x:%.1f y:%.1f | tags:%d | range:%.1f in",
                        center[0], center[1], c.tagCount, c.rangeMeters() * METERS_TO_INCHES);
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        limelight.stop();
    }

    private static double deadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0.0;
    }

    private static ClusterDetection closestToCameraAxis(List<ClusterDetection> clusters) {
        ClusterDetection best = null;
        for (ClusterDetection c : clusters) {
            if (best == null || Math.abs(c.bearingDeg()) < Math.abs(best.bearingDeg())) {
                best = c;
            }
        }
        return best;
    }

    /**
     * Robot heading (radians) that puts the field point (cx, cy) on the camera's optical axis when
     * the robot turns in place about its center. The camera line of sight is offset from the robot
     * center by d (its perpendicular distance), so the heading is the bearing to the point minus
     * the camera yaw, corrected by asin(d / distance).
     */
    private static double headingToLookAt(Pose robot, double cx, double cy) {
        double dx = cx - robot.x();
        double dy = cy - robot.y();
        double distance = Math.hypot(dx, dy);
        double yaw = Math.toRadians(CAMERA_YAW_DEG);
        double offset = -CAMERA_FORWARD_IN * Math.sin(yaw) + CAMERA_LEFT_IN * Math.cos(yaw);
        if (distance <= Math.abs(offset) + 1e-6) {
            return robot.heading();  // cluster is closer than the camera's line of sight can reach
        }
        return normalizeAngle(Math.atan2(dy, dx) - yaw - Math.asin(offset / distance));
    }

    /** Wraps an angle to (-pi, pi]. */
    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Converts a Limelight camera-space point (meters) or direction to the Pedro field frame (inches).
     *
     * @param isPoint true to apply the camera mount offset and robot pose translation, false for a direction
     * @return {fieldX, fieldY}
     */
    private static double[] cameraToField(double cx, double cy, double cz, boolean isPoint, Pose robot) {
        double scale = isPoint ? METERS_TO_INCHES : 1.0;

        // Camera space (X right, Y down, Z forward) -> camera-aligned robot frame (forward, left)
        double forward = cz * scale;
        double left = -cx * scale;

        // Camera yaw relative to the robot
        double yaw = Math.toRadians(CAMERA_YAW_DEG);
        double robotForward = forward * Math.cos(yaw) - left * Math.sin(yaw);
        double robotLeft = forward * Math.sin(yaw) + left * Math.cos(yaw);

        if (isPoint) {
            robotForward += CAMERA_FORWARD_IN;
            robotLeft += CAMERA_LEFT_IN;
        }

        // Robot frame -> field frame
        double h = robot.heading();
        double fx = robotForward * Math.cos(h) - robotLeft * Math.sin(h);
        double fy = robotForward * Math.sin(h) + robotLeft * Math.cos(h);
        if (isPoint) {
            fx += robot.x();
            fy += robot.y();
        }
        return new double[]{fx, fy};
    }
}
