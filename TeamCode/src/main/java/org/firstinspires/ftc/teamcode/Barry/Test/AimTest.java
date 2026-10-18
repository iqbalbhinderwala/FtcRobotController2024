package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Barry.Auto.HiveAuto;
import org.firstinspires.ftc.teamcode.Barry.Auto.StartPosition;
import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.Barry.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Barry.Vision.CameraMount;
import org.firstinspires.ftc.teamcode.Barry.Vision.ClusterDetection;
import org.firstinspires.ftc.teamcode.Barry.Vision.LimelightAprilTagClusterDetector;

import java.util.List;

/**
 * Drives the robot field-centric (Pedro ManualDrive.fieldCentric) and draws the AprilTag clusters seen by the Limelight
 * on the FTC Dashboard field, in the Pedro frame (inches).
 *
 * Camera space (Limelight): X right, Y down, Z forward. It is converted to the robot frame and the
 * field with CameraMount (position, tilt, yaw; live-editable from FTC Dashboard).
 *
 * Start: same as HiveAuto. Flush on the wall (blue: far wall, heading 180; red: audience wall,
 * heading 0). During INIT the position along the wall comes from the alliance's audience-side
 * cluster (blue 42-45, red 34-37) when it is seen, otherwise the B1 spot is assumed.
 *
 * Controls (gamepad1): left stick drives field-centric, right stick rotates,
 * A turns the robot in place (follower.hold) so the camera looks at the center of the cluster nearest
 * the camera axis; press again to refine. Any stick input cancels.
 */
@Config
@TeleOp(name = "Aim Test", group = "BarryTest")
public class AimTest extends LinearOpMode {

    /**
     * Start on the blue side (far wall) like the blue Auto; false = red (audience wall). On blue the
     * Dashboard field is drawn turned 180 degrees (Drawing.BLUE_PERSPECTIVE) while this OpMode runs.
     */
    public static boolean BLUE = true;

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

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dual telemetry for Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Follower follower = Constants.create(hardwareMap);
        boolean previousPerspective = Drawing.BLUE_PERSPECTIVE;

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightAprilTagClusterDetector detector = new LimelightAprilTagClusterDetector();
        limelight.pipelineSwitch(0);
        limelight.start();

        // INIT: start pose from the audience-side cluster, like the Auto (robot does not move)
        while (!isStarted() && !isStopRequested()) {
            Drawing.BLUE_PERSPECTIVE = BLUE;
            ClusterDetection audience = detector.find(detector.detect(limelight.getLatestResult()),
                    StartPosition.audienceClusterId(BLUE));
            double redX = audience != null ? StartPosition.redXFromCamera(audience) : HiveAuto.B1_X;
            follower.setPose(StartPosition.pose(redX, BLUE));

            Pose start = follower.pose();
            telemetry.addData("Side", BLUE ? "blue (far wall)" : "red (audience wall)");
            telemetry.addData("Start", "x:%.1f y:%.1f heading:%.1f deg (%s)", start.x(), start.y(),
                    Math.toDegrees(start.heading()), audience != null ? "from camera" : "assumed B1");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
            Drawing.drawRobot(packet.fieldOverlay(), start);
            dashboard.sendTelemetryPacket(packet);
        }

        boolean lastA = false;
        String aimTarget = null;        // name of the cluster being aimed at, null when not aiming
        double aimHeading = 0.0;        // target robot heading, radians
        String aimStatus = "idle";

        while (opModeIsActive()) {
            Drawing.BLUE_PERSPECTIVE = BLUE;
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
                    double[] center = CameraMount.toField(nearest.x, nearest.y, nearest.z, robot);
                    aimTarget = nearest.name;
                    aimHeading = CameraMount.headingToLookAt(robot, center[0], center[1]);
                    follower.hold(new Pose(robot.x(), robot.y(), aimHeading));
                }
            }
            lastA = a;
            if (hasManualInput && aimTarget != null) {
                aimTarget = null;
                aimStatus = "cancelled";
            }
            if (aimTarget != null) {
                double error = Math.toDegrees(CameraMount.normalize(aimHeading - robot.heading()));
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
                double[] center = CameraMount.toField(c.x, c.y, c.z, robot);
                double[] dir = CameraMount.directionToField(c.dirX, c.dirY, c.dirZ, robot);
                double dirLength = Math.hypot(dir[0], dir[1]);
                if (dirLength > 1e-6) {
                    dir[0] /= dirLength;
                    dir[1] /= dirLength;
                }

                String color = c.name.startsWith("RED") ? "red" : "blue";
                Drawing.drawCluster(packet.fieldOverlay(), center[0], center[1], dir[0], dir[1],
                        color, c.name + " (" + c.tagCount + ")");

                telemetry.addData(c.name, "field x:%.1f y:%.1f z:%.1f | tags:%d | range:%.1f in",
                        center[0], center[1], center[2], c.tagCount, c.rangeMeters() * METERS_TO_INCHES);
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        limelight.stop();
        Drawing.BLUE_PERSPECTIVE = previousPerspective;
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
}
