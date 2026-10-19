package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Barry.Auto.HiveAuto;
import org.firstinspires.ftc.teamcode.Barry.Auto.InitLocalizer;
import org.firstinspires.ftc.teamcode.Barry.Auto.StartPosition;
import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.Barry.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Barry.Util.CsvLog;
import org.firstinspires.ftc.teamcode.Barry.Vision.CameraMount;
import org.firstinspires.ftc.teamcode.Barry.Vision.ClusterDetection;
import org.firstinspires.ftc.teamcode.Barry.Vision.LimelightAprilTagClusterDetector;

import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Tests InitLocalizer. Every HIVE cluster in view (30-33, 34-37, 38-41, 42-45) predicts the robot
 * pose on its own, using its HIVE's tilt (TILT_MODE, default SNAP: the stable +/-30 whose height is
 * closer to the measured one). Only clusters whose visible tags span InitLocalizer.MIN_SPAN_IN are
 * used. Each prediction is drawn on the Dashboard field as a robot outline in its own color, next to
 * the fused pose; the alliance follows from the field half (red = Pedro x < 72). In INIT the fused
 * pose is snapped to the wall when the robot is close to parallel (InitLocalizer.SNAP_TO_WALL).
 *
 * INIT: keep the robot still. For each cluster, telemetry shows its average prediction over the last
 * WINDOW frames, its own spread, its distance from the fused pose, its solved tilt and its height.
 * The robot is drawn at the fused pose (field turned 180 deg for blue).
 * START: odometry starts from the fused INIT pose. Drive robot-centric with gamepad1 (left stick
 * drive, right stick turn). The robot is drawn at odometry, the fused camera pose as a lime outline
 * and each cluster's prediction as an outline in its color; telemetry shows camera - odometry.
 *
 * Outline colors: 30-33 orange, 34-37 red, 38-41 cyan, 42-45 blue, fused lime.
 * Logged to /sdcard/FIRST/barry_logs/init_localizer_<time>.csv (one row per loop and per cluster).
 */
@Config
@TeleOp(name = "Init Localizer Test", group = "BarryTest")
public class InitLocalizerTest extends LinearOpMode {

    public static InitLocalizer.TiltMode TILT_MODE = InitLocalizer.TiltMode.SNAP;
    public static double DRIVE_SCALE = 0.4;
    public static double TURN_SCALE = 0.3;

    private static final double STICK_DEADBAND = 0.05;
    private static final String LOG_HEADER = "t_s,phase,source,tags,span_in,range_in,height_in,tilt_deg,"
            + "x,y,h_deg,fused_x,fused_y,fused_h_deg,odo_x,odo_y,odo_h_deg";

    /** Recent predictions of one cluster while the robot is still (INIT). */
    private final Map<Integer, ArrayDeque<Pose>> history = new LinkedHashMap<>();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        boolean previousPerspective = Drawing.BLUE_PERSPECTIVE;
        CsvLog log = new CsvLog("init_localizer", LOG_HEADER);
        ElapsedTime timer = new ElapsedTime();

        Follower follower = Constants.create(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightAprilTagClusterDetector detector = new LimelightAprilTagClusterDetector();
        InitLocalizer localizer = new InitLocalizer();
        limelight.pipelineSwitch(0);
        limelight.start();

        // ---- INIT: robot still; per-cluster predictions and their precision ----
        while (!isStarted() && !isStopRequested()) {
            List<ClusterDetection> clusters = detector.detect(limelight.getLatestResult());
            localizer.update(clusters, TILT_MODE);
            InitLocalizer.Estimate frame = localizer.latest();
            Pose fused = localizer.pose();
            remember(frame);

            telemetry.addData("Alliance", localizer.hasFix() ? localizer.alliance() : "no HIVE cluster seen yet");
            telemetry.addData("Tilt mode", TILT_MODE);
            if (fused != null) {
                Pose raw = localizer.rawAverage();
                telemetry.addData("Fused", "x:%.1f y:%.1f h:%.1f deg%s | %d frames, spread %.2f in / %.2f deg",
                        fused.x(), fused.y(), Math.toDegrees(fused.heading()),
                        localizer.snappedToWall() ? String.format(" (wall snap from %.1f, %.1f, %.1f)",
                                raw.x(), raw.y(), Math.toDegrees(raw.heading())) : "",
                        localizer.samples(), localizer.positionSpreadIn(), localizer.headingSpreadDeg());
                follower.setPose(fused);
            }
            addClusterPrecision(frame, fused);
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();

            Drawing.BLUE_PERSPECTIVE = localizer.alliance() == HiveAuto.Alliance.BLUE;
            TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
            Drawing.drawRobot(packet.fieldOverlay(), fused);
            drawPredictions(packet.fieldOverlay(), frame);
            dashboard.sendTelemetryPacket(packet);

            logFrame(log, timer, "init", frame, fused, null);
        }

        if (!localizer.hasFix()) {
            // No cluster seen: fall back to the red B1 start so odometry has something sensible
            follower.setPose(StartPosition.pose(HiveAuto.B1_X, false));
        }

        // ---- START: drive around, compare odometry with the camera ----
        while (opModeIsActive()) {
            double forward = deadband(-gamepad1.left_stick_y) * DRIVE_SCALE;
            double strafe = deadband(-gamepad1.left_stick_x) * DRIVE_SCALE;
            double turn = deadband(-gamepad1.right_stick_x) * TURN_SCALE;
            follower.manual(forward, strafe, turn);
            follower.update();

            Pose odo = follower.pose();
            InitLocalizer.Estimate frame = InitLocalizer.fromFrame(
                    detector.detect(limelight.getLatestResult()), TILT_MODE);

            telemetry.addData("Odometry", "x:%.1f y:%.1f h:%.1f deg", odo.x(), odo.y(), Math.toDegrees(odo.heading()));
            if (frame != null) {
                Pose cam = frame.pose;
                telemetry.addData("Fused camera", "x:%.1f y:%.1f h:%.1f deg (%s)", cam.x(), cam.y(),
                        Math.toDegrees(cam.heading()), frame.alliance);
                telemetry.addData("Fused - odometry", "dx:%+.1f dy:%+.1f dh:%+.1f deg", cam.x() - odo.x(),
                        cam.y() - odo.y(), Math.toDegrees(CameraMount.normalize(cam.heading() - odo.heading())));
                for (InitLocalizer.ClusterPrediction p : frame.used) {
                    telemetry.addData(name(p.firstTagId) + " - odometry",
                            "dx:%+.1f dy:%+.1f dh:%+.1f deg | %d tags, tilt %+.0f",
                            p.pose.x() - odo.x(), p.pose.y() - odo.y(),
                            Math.toDegrees(CameraMount.normalize(p.pose.heading() - odo.heading())),
                            p.tags, p.tiltDeg);
                }
            } else {
                telemetry.addData("Camera", "no HIVE cluster in view");
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
            Drawing.drawRobot(packet.fieldOverlay(), odo);
            drawPredictions(packet.fieldOverlay(), frame);
            dashboard.sendTelemetryPacket(packet);

            logFrame(log, timer, "drive", frame, frame != null ? frame.pose : null, odo);
        }

        log.close();
        limelight.stop();
        Drawing.BLUE_PERSPECTIVE = previousPerspective;
    }

    // =========================================================================
    // Per-cluster precision (INIT, robot still)
    // =========================================================================

    private void remember(InitLocalizer.Estimate frame) {
        if (frame == null) {
            return;
        }
        for (InitLocalizer.ClusterPrediction p : frame.used) {
            ArrayDeque<Pose> poses = history.get(p.firstTagId);
            if (poses == null) {
                poses = new ArrayDeque<>();
                history.put(p.firstTagId, poses);
            }
            poses.addLast(p.pose);
            while (poses.size() > Math.max(1, InitLocalizer.WINDOW)) {
                poses.removeFirst();
            }
        }
    }

    /** Per cluster: average prediction, its own spread, distance from the fused pose, tilt, height. */
    private void addClusterPrecision(InitLocalizer.Estimate frame, Pose fused) {
        for (Map.Entry<Integer, ArrayDeque<Pose>> entry : history.entrySet()) {
            int id = entry.getKey();
            ArrayDeque<Pose> poses = entry.getValue();
            Pose mean = mean(poses);
            InitLocalizer.ClusterPrediction now = find(frame, id);
            String fromFused = fused == null ? "" : String.format(" | from fused %.2f in / %+.1f deg",
                    Math.hypot(mean.x() - fused.x(), mean.y() - fused.y()),
                    Math.toDegrees(CameraMount.normalize(mean.heading() - fused.heading())));
            String state = now == null ? " | not used now" : String.format(" | %d tags, span %.1f, tilt %+.0f, z %.1f in",
                    now.tags, now.spanIn, now.tiltDeg, now.heightIn);
            telemetry.addData(name(id), "x:%.1f y:%.1f h:%.1f deg | spread %.2f in / %.2f deg%s%s",
                    mean.x(), mean.y(), Math.toDegrees(mean.heading()),
                    spreadIn(poses, mean), spreadDeg(poses, mean), fromFused, state);
        }
    }

    private static Pose mean(ArrayDeque<Pose> poses) {
        double sx = 0, sy = 0, ss = 0, sc = 0;
        for (Pose p : poses) {
            sx += p.x();
            sy += p.y();
            ss += Math.sin(p.heading());
            sc += Math.cos(p.heading());
        }
        return new Pose(sx / poses.size(), sy / poses.size(), Math.atan2(ss, sc));
    }

    private static double spreadIn(ArrayDeque<Pose> poses, Pose mean) {
        double sum = 0;
        for (Pose p : poses) {
            double dx = p.x() - mean.x();
            double dy = p.y() - mean.y();
            sum += dx * dx + dy * dy;
        }
        return Math.sqrt(sum / poses.size());
    }

    private static double spreadDeg(ArrayDeque<Pose> poses, Pose mean) {
        double sum = 0;
        for (Pose p : poses) {
            double d = CameraMount.normalize(p.heading() - mean.heading());
            sum += d * d;
        }
        return Math.toDegrees(Math.sqrt(sum / poses.size()));
    }

    // =========================================================================
    // Drawing, logging, helpers
    // =========================================================================

    /** A robot outline per cluster prediction in its color, plus the fused pose in lime. */
    private static void drawPredictions(Canvas field, InitLocalizer.Estimate frame) {
        if (frame == null) {
            return;
        }
        for (InitLocalizer.ClusterPrediction p : frame.used) {
            Drawing.drawRobotOutline(field, p.pose, color(p.firstTagId), String.valueOf(p.firstTagId));
        }
        Drawing.drawRobotOutline(field, frame.pose, "lime", "fused");
    }

    private static void logFrame(CsvLog log, ElapsedTime timer, String phase, InitLocalizer.Estimate frame,
                                 Pose fused, Pose odo) {
        Object fx = fused != null ? fused.x() : null;
        Object fy = fused != null ? fused.y() : null;
        Object fh = fused != null ? Math.toDegrees(fused.heading()) : null;
        Object ox = odo != null ? odo.x() : null;
        Object oy = odo != null ? odo.y() : null;
        Object oh = odo != null ? Math.toDegrees(odo.heading()) : null;
        if (frame == null) {
            log.row(timer.seconds(), phase, "none", null, null, null, null, null, null, null, null, fx, fy, fh, ox, oy, oh);
            return;
        }
        for (InitLocalizer.ClusterPrediction p : frame.used) {
            log.row(timer.seconds(), phase, p.firstTagId, p.tags, p.spanIn, p.rangeIn, p.heightIn, p.tiltDeg,
                    p.pose.x(), p.pose.y(), Math.toDegrees(p.pose.heading()), fx, fy, fh, ox, oy, oh);
        }
        log.row(timer.seconds(), phase, "fused", frame.tags(), null, null, null, null,
                frame.pose.x(), frame.pose.y(), Math.toDegrees(frame.pose.heading()), fx, fy, fh, ox, oy, oh);
    }

    private static InitLocalizer.ClusterPrediction find(InitLocalizer.Estimate frame, int firstTagId) {
        if (frame == null) {
            return null;
        }
        for (InitLocalizer.ClusterPrediction p : frame.used) {
            if (p.firstTagId == firstTagId) return p;
        }
        return null;
    }

    private static String name(int firstTagId) {
        return firstTagId + "-" + (firstTagId + 3);
    }

    private static String color(int firstTagId) {
        switch (firstTagId) {
            case 30: return "orange";
            case 34: return "red";
            case 38: return "cyan";
            default: return "blue";
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) > STICK_DEADBAND ? value : 0.0;
    }
}
