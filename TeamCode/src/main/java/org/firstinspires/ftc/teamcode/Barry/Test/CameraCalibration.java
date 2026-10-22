package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Barry.Auto.HiveAuto;
import org.firstinspires.ftc.teamcode.Barry.Auto.InitLocalizer;
import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.Barry.Field.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.Barry.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Barry.Pedro.FtcToPedro;
import org.firstinspires.ftc.teamcode.Barry.Util.CsvLog;
import org.firstinspires.ftc.teamcode.Barry.Vision.CameraMount;
import org.firstinspires.ftc.teamcode.Barry.Vision.LimelightAprilTagClusterDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Camera calibration: drives a pattern of stops in front of the HIVE and logs every tag the
 * Limelight sees, so the camera mount (offset, yaw, tilt, roll, distance scale) and the HIVE offset
 * can be solved offline (TeamCode/.../Camera/python/limelight_extrinsics.py).
 *
 * Setup: robot parallel to a wall with its right side against it, camera (left side) facing its
 * HIVE, HIVE not moving. Only the heading has to be exact; the start position and the HIVE position
 * may each be off by a couple of inches (they are solved, relative to each other).
 *
 * INIT: InitLocalizer finds the start pose (heading snapped to the wall). START: the robot first logs
 * at the start, then drives the pattern relative to it: rows ROW*_OFF_IN off the wall, at -ALONG_IN /
 * 0 / +ALONG_IN along it, and at each spot the heading that points the camera at its HIVE plus 0,
 * +/-TURN1_DEG, +/-TURN2_DEG (so the HIVE moves across the image). Turns only happen away from the
 * wall. With KEEP_IN_REGION the whole robot footprint stays inside REGION_* (blue, Pedro; default
 * columns D-F, rows 5-6; mirrored for red): per row, the spots along the wall shift as little as
 * needed to fit. Stops where neither cluster of the HIVE would be in view (VIEW_*_DEG, nominal HIVE at
 * HIVE_TILT_DEG, current CameraMount) are dropped. At each stop it settles for SETTLE_MS, then logs
 * the tags for COLLECT_MS. Planned for a start centered in D6 against the far wall (blue); any
 * start against a wall facing the robot's own HIVE works.
 * Approach speed is capped, Pedro's hold does the last bit. gamepad1 B aborts (holds where it is);
 * STEP_MODE waits for gamepad1 Y before each move.
 *
 * Logged to /sdcard/FIRST/barry_logs/r<run>_camera_calibration_<time>.csv: one row per tag per
 * camera frame (raw camera-space position) with the odometry pose and stop, plus event rows.
 */
@Config
@TeleOp(name = "Camera Calibration", group = "BarryTest")
public class CameraCalibration extends LinearOpMode {

    // ---- Pattern, relative to the start: along = robot forward, off = robot left (off the wall) ----
    public static double ROW1_OFF_IN = 4.0;
    public static double ROW2_OFF_IN = 10.0;
    public static double ROW3_OFF_IN = 16.0;
    public static double ROW4_OFF_IN = 22.0;
    public static double ALONG_IN = 10.0;
    /** Turns away from the heading that points the camera at the HIVE. */
    public static double TURN1_DEG = 12.0;
    public static double TURN2_DEG = 22.0;

    // ---- Keep only stops where a cluster is expected in view (with a margin inside the image) ----
    public static boolean REQUIRE_VIEW = true;
    public static double VIEW_H_DEG = 24.0;
    public static double VIEW_V_DEG = 18.0;
    /** HIVE tilt during calibration (alliance convention, +30 = match start). */
    public static double HIVE_TILT_DEG = 30.0;

    // ---- Area the robot must stay in (whole footprint), Pedro, for blue; mirrored for red ----
    public static boolean KEEP_IN_REGION = true;
    public static double REGION_MIN_X = 72.0;   // columns D-F
    public static double REGION_MAX_X = 144.0;
    public static double REGION_MIN_Y = 96.0;   // rows 5-6
    public static double REGION_MAX_Y = 144.0;
    /** Keep this far inside the region (the start, against the wall, is exempt). */
    public static double REGION_MARGIN_IN = 0.5;

    // ---- Driving to a stop: capped approach, then Pedro hold for the last bit ----
    public static double MAX_DRIVE_POWER = 0.3;
    public static double MAX_TURN_POWER = 0.3;
    /** Power per inch / per radian of error during the approach. */
    public static double KP_DRIVE = 0.05;
    public static double KP_TURN = 1.0;
    public static double HOLD_WITHIN_IN = 2.0;
    public static double HOLD_WITHIN_DEG = 4.0;
    public static double ARRIVED_IN = 0.75;
    public static double ARRIVED_DEG = 1.5;
    public static double STOP_TIMEOUT_S = 8.0;

    // ---- Collecting ----
    public static double SETTLE_MS = 600;
    public static double COLLECT_MS = 1500;
    /** Wait for gamepad1 Y before moving to each stop. */
    public static boolean STEP_MODE = false;

    private static final String LOG_HEADER = "t_s,state,stop,along_in,off_in,turn_deg,target_x,target_y,target_h_deg,"
            + "odo_x,odo_y,odo_h_deg,frame_ts,tag_id,cam_x_m,cam_y_m,cam_z_m,tx_deg,ty_deg,"
            + "tag_roll_deg,tag_pitch_deg,tag_yaw_deg,event";

    private enum State { WAIT_Y, MOVE, HOLD, SETTLE, COLLECT, DONE, ABORTED }

    private CsvLog log;
    private Follower follower;
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private Pose start;
    private List<double[]> stops;
    private int index;
    private double[] stop;
    private Pose target;
    private State state;
    private int framesThisStop;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        log = new CsvLog("camera_calibration", LOG_HEADER);

        follower = Constants.create(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightAprilTagClusterDetector detector = new LimelightAprilTagClusterDetector();
        InitLocalizer localizer = new InitLocalizer();
        limelight.pipelineSwitch(0);
        limelight.start();
        stops = new ArrayList<>();

        // ---- INIT: start pose from the HIVE (heading snapped to the wall) ----
        while (!isStarted() && !isStopRequested()) {
            localizer.update(detector.detect(limelight.getLatestResult()));
            Pose estimate = localizer.pose();
            if (estimate != null) {
                follower.setPose(estimate);
                telemetry.addData("Start", "x:%.1f y:%.1f h:%.1f deg (%s)", estimate.x(), estimate.y(),
                        Math.toDegrees(estimate.heading()),
                        localizer.snappedToWall() ? "heading snapped to the wall" : "NOT parallel to a wall");
            } else {
                telemetry.addData("Start", "no HIVE cluster seen yet");
            }
            if (estimate != null) {
                stops = stops(estimate);
            }
            telemetry.addData("Pattern", "%d stops%s; B aborts%s", stops.size(),
                    KEEP_IN_REGION ? " inside the region" : "", STEP_MODE ? ", Y starts each move" : "");
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();
        }
        if (isStopRequested()) {
            close(limelight);
            return;
        }

        start = localizer.pose();
        if (start == null || !localizer.snappedToWall()) {
            event("no calibration: start pose unknown or not parallel to a wall");
            while (opModeIsActive()) {
                telemetry.addData("Camera Calibration", "Not started: HIVE not seen, or the robot is not parallel to a wall.");
                telemetry.update();
            }
            close(limelight);
            return;
        }
        follower.setPose(start);
        stops = stops(start);
        clock.reset();
        event("pattern: " + describe(stops));
        event(String.format(Locale.US,
                "settings: start (%.2f;%.2f;%.2f deg) camera forward %.3f left %.3f height %.3f yaw %.3f pitch %.3f roll %.3f scale %.4f",
                start.x(), start.y(), Math.toDegrees(start.heading()), CameraMount.FORWARD_IN, CameraMount.LEFT_IN,
                CameraMount.HEIGHT_IN, CameraMount.YAW_DEG, CameraMount.PITCH_DEG, CameraMount.ROLL_DEG,
                CameraMount.DISTANCE_SCALE));

        // The first stop is the start itself: log there before moving
        index = 0;
        stop = stops.get(0);
        target = targetPose(start, stop);
        follower.hold(target);
        setState(State.SETTLE);

        double lastFrame = -1;
        boolean lastY = false;

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.pose();
            boolean y = gamepad1.y;
            boolean yPressed = y && !lastY;
            lastY = y;

            if (gamepad1.b && state != State.DONE && state != State.ABORTED) {
                follower.hold(pose);
                setState(State.ABORTED);
                event("aborted by driver");
            }

            double[] err = error(target, pose);
            double distance = Math.hypot(err[0], err[1]);
            switch (state) {
                case WAIT_Y:
                    if (yPressed) setState(State.MOVE);
                    break;
                case MOVE:
                    if (distance <= HOLD_WITHIN_IN && Math.abs(err[2]) <= Math.toRadians(HOLD_WITHIN_DEG)) {
                        follower.hold(target);
                        setState(State.HOLD);
                    } else if (stateTimer.seconds() > STOP_TIMEOUT_S) {
                        advance("timeout while moving: stop skipped");
                    } else {
                        follower.manual(clamp(KP_DRIVE * err[0], MAX_DRIVE_POWER),
                                clamp(KP_DRIVE * err[1], MAX_DRIVE_POWER),
                                clamp(KP_TURN * err[2], MAX_TURN_POWER));
                    }
                    break;
                case HOLD:
                    if (distance <= ARRIVED_IN && Math.abs(err[2]) <= Math.toRadians(ARRIVED_DEG)) {
                        event("arrived");
                        setState(State.SETTLE);
                    } else if (stateTimer.seconds() > STOP_TIMEOUT_S) {
                        advance("timeout while holding: stop skipped");
                    }
                    break;
                case SETTLE:
                    if (stateTimer.milliseconds() >= SETTLE_MS) {
                        framesThisStop = 0;
                        setState(State.COLLECT);
                    }
                    break;
                case COLLECT:
                    if (stateTimer.milliseconds() >= COLLECT_MS) {
                        advance("collected " + framesThisStop + " frames with tags");
                    }
                    break;
                default:
                    break;
            }

            // Log every new camera frame while settling and collecting (raw, per tag)
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getTimestamp() != lastFrame
                    && (state == State.SETTLE || state == State.COLLECT)) {
                lastFrame = result.getTimestamp();
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty() && state == State.COLLECT) framesThisStop++;
                for (LLResultTypes.FiducialResult tag : tags) {
                    logTag(pose, lastFrame, tag);
                }
            }

            telemetry.addData("Stop", "%d / %d  (along %+.0f, off %.0f, turn %+.0f)", index + 1, stops.size(),
                    stop[0], stop[1], stop[2]);
            telemetry.addData("State", "%s%s", state, state == State.WAIT_Y ? "  (press Y)" : "");
            telemetry.addData("Error", "%.2f in, %.2f deg", distance, Math.toDegrees(err[2]));
            telemetry.addData("Frames this stop", framesThisStop);
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            Drawing.drawRobotOutline(packet.fieldOverlay(), target, "lime", String.valueOf(index + 1));
            dashboard.sendTelemetryPacket(packet);
        }

        close(limelight);
    }

    private void setState(State next) {
        state = next;
        stateTimer.reset();
    }

    /** Go to the next stop (or finish), logging why. */
    private void advance(String why) {
        event(why);
        if (index + 1 >= stops.size()) {
            follower.hold(follower.pose());
            setState(State.DONE);
            event("done");
            return;
        }
        index++;
        stop = stops.get(index);
        target = targetPose(start, stop);
        setState(STEP_MODE ? State.WAIT_Y : State.MOVE);
    }

    /**
     * {along, off, turn}: the start, then per row 3 spots along the wall x 5 headings. The turn is
     * relative to the start heading; at each spot the headings are the one that points the camera at
     * the HIVE plus 0, +/-TURN1_DEG, +/-TURN2_DEG. Spots shift along the wall to stay in the region,
     * and stops outside the region or without a cluster in view are dropped.
     */
    private static List<double[]> stops(Pose start) {
        List<double[]> out = new ArrayList<>();
        out.add(new double[]{0, 0, 0});
        double[] rows = {ROW1_OFF_IN, ROW2_OFF_IN, ROW3_OFF_IN, ROW4_OFF_IN};
        for (double off : rows) {
            for (double along : alongs(start, off)) {
                for (double turn : turnsAt(start, along, off)) {
                    double[] stop = {along, off, turn};
                    Pose pose = targetPose(start, stop);
                    if ((!KEEP_IN_REGION || inRegion(start, pose)) && (!REQUIRE_VIEW || clusterInView(start, pose))) {
                        out.add(stop);
                    }
                }
            }
        }
        return out;
    }

    /** Turns (relative to the start heading) at a spot: aim at the HIVE, then +/-TURN1, +/-TURN2. */
    private static double[] turnsAt(Pose start, double along, double off) {
        Pose spot = targetPose(start, new double[]{along, off, 0});
        double[] hive = hiveCenter(start);
        double aim = Math.toDegrees(CameraMount.normalize(
                CameraMount.headingToLookAt(spot, hive[0], hive[1]) - start.heading()));
        return new double[]{aim, aim + TURN1_DEG, aim + TURN2_DEG, aim - TURN1_DEG, aim - TURN2_DEG};
    }

    /** Spots along the wall for one row: -A / 0 / +A, shifted as little as needed to fit, else spread over what fits. */
    private static double[] alongs(Pose start, double off) {
        double[] wanted = {-ALONG_IN, 0, ALONG_IN};
        if (!KEEP_IN_REGION) {
            return wanted;
        }
        double lo = Double.NaN;
        double hi = Double.NaN;
        for (double along = -40.0; along <= 40.0; along += 0.5) {
            boolean fits = true;
            for (double turn : turnsAt(start, along, off)) {
                fits &= inRegion(start, targetPose(start, new double[]{along, off, turn}));
            }
            if (fits) {
                if (Double.isNaN(lo)) lo = along;
                hi = along;
            }
        }
        if (Double.isNaN(lo)) {
            return wanted;   // nothing fits at every heading: keep the plan, stops() drops what doesn't fit
        }
        if (hi - lo >= 2.0 * ALONG_IN) {
            // Shift the usual spots as little as possible to fit
            double center = Math.max(lo + ALONG_IN, Math.min(hi - ALONG_IN, 0.0));
            return new double[]{center - ALONG_IN, center, center + ALONG_IN};
        }
        return new double[]{lo, (lo + hi) / 2.0, hi};
    }

    /** Pedro center (pivot line) of the HIVE the robot faces at the start: its own alliance's. */
    private static double[] hiveCenter(Pose start) {
        boolean red = start.x() < 72.0;
        return FtcToPedro.point(0.0, (red ? -1.0 : 1.0) * HiveTagGeometry.HIVE_CENTER_TO_CENTER / 2.0);
    }

    /** True when either cluster of the facing HIVE (nominal, HIVE_TILT_DEG) is inside the view margin. */
    private static boolean clusterInView(Pose start, Pose pose) {
        boolean red = start.x() < 72.0;
        for (boolean audience : new boolean[]{true, false}) {
            HiveTagGeometry.TagPose c = HiveTagGeometry.clusterPose(red, audience, HIVE_TILT_DEG);
            double[] p = FtcToPedro.point(c.x, c.y);
            // Field -> robot -> camera-level frame (undo the mount offset and yaw), then angles
            double dx = p[0] - pose.x();
            double dy = p[1] - pose.y();
            double rf = dx * Math.cos(pose.heading()) + dy * Math.sin(pose.heading()) - CameraMount.FORWARD_IN;
            double rl = -dx * Math.sin(pose.heading()) + dy * Math.cos(pose.heading()) - CameraMount.LEFT_IN;
            double yaw = Math.toRadians(CameraMount.YAW_DEG);
            double ahead = rf * Math.cos(yaw) + rl * Math.sin(yaw);
            double left = -rf * Math.sin(yaw) + rl * Math.cos(yaw);
            double up = c.z - CameraMount.HEIGHT_IN;
            double side = Math.toDegrees(Math.atan2(left, ahead));
            double elevation = Math.toDegrees(Math.atan2(up, Math.hypot(ahead, left))) - CameraMount.PITCH_DEG;
            if (ahead > 0 && Math.abs(side) <= VIEW_H_DEG && Math.abs(elevation) <= VIEW_V_DEG) {
                return true;
            }
        }
        return false;
    }

    /** True when the robot footprint at this pose is inside the region (mirrored for red). */
    private static boolean inRegion(Pose start, Pose pose) {
        boolean red = start.x() < 72.0;
        double minX = red ? 144.0 - REGION_MAX_X : REGION_MIN_X;
        double maxX = red ? 144.0 - REGION_MIN_X : REGION_MAX_X;
        double minY = red ? 144.0 - REGION_MAX_Y : REGION_MIN_Y;
        double maxY = red ? 144.0 - REGION_MIN_Y : REGION_MAX_Y;
        double c = Math.cos(pose.heading());
        double s = Math.sin(pose.heading());
        double halfLength = HiveAuto.ROBOT_HALF_LENGTH;
        double halfWidth = HiveAuto.ROBOT_HALF_WIDTH;
        for (int i = 0; i < 4; i++) {
            double a = (i < 2 ? 1 : -1) * halfLength;
            double b = (i % 2 == 0 ? 1 : -1) * halfWidth;
            double x = pose.x() + a * c - b * s;
            double y = pose.y() + a * s + b * c;
            if (x < minX + REGION_MARGIN_IN || x > maxX - REGION_MARGIN_IN
                    || y < minY + REGION_MARGIN_IN || y > maxY - REGION_MARGIN_IN) {
                return false;
            }
        }
        return true;
    }

    private static String describe(List<double[]> stops) {
        StringBuilder sb = new StringBuilder(stops.size() + " stops (along;off;turn)");
        for (double[] st : stops) {
            sb.append(String.format(Locale.US, " %.1f;%.0f;%.0f", st[0], st[1], st[2]));
        }
        return sb.toString();
    }

    /** Field pose of a stop: the start moved along / off in the start's own frame and turned. */
    private static Pose targetPose(Pose start, double[] stop) {
        double c = Math.cos(start.heading());
        double s = Math.sin(start.heading());
        return new Pose(start.x() + stop[0] * c - stop[1] * s,
                start.y() + stop[0] * s + stop[1] * c,
                CameraMount.normalize(start.heading() + Math.toRadians(stop[2])));
    }

    /** Target minus pose in the robot frame: {forward, left, heading (rad)}. */
    private static double[] error(Pose target, Pose pose) {
        double dx = target.x() - pose.x();
        double dy = target.y() - pose.y();
        double h = pose.heading();
        return new double[]{
                dx * Math.cos(h) + dy * Math.sin(h),
                -dx * Math.sin(h) + dy * Math.cos(h),
                CameraMount.normalize(target.heading() - h)};
    }

    private static double clamp(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }

    private void logTag(Pose pose, double frameTs, LLResultTypes.FiducialResult tag) {
        Pose3D p = tag.getTargetPoseCameraSpace();
        Object cx = null, cy = null, cz = null, roll = null, pitch = null, yaw = null;
        if (p != null) {
            cx = p.getPosition().toUnit(DistanceUnit.METER).x;
            cy = p.getPosition().toUnit(DistanceUnit.METER).y;
            cz = p.getPosition().toUnit(DistanceUnit.METER).z;
            roll = p.getOrientation().getRoll(AngleUnit.DEGREES);
            pitch = p.getOrientation().getPitch(AngleUnit.DEGREES);
            yaw = p.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        log.row(clock.seconds(), state, index, stop[0], stop[1], stop[2], target.x(), target.y(),
                Math.toDegrees(target.heading()), pose.x(), pose.y(), Math.toDegrees(pose.heading()), frameTs,
                tag.getFiducialId(), cx, cy, cz, tag.getTargetXDegrees(), tag.getTargetYDegrees(),
                roll, pitch, yaw, null);
    }

    private void event(String text) {
        Pose pose = follower.pose();
        log.row(clock.seconds(), "event", index, stop != null ? stop[0] : null, stop != null ? stop[1] : null,
                stop != null ? stop[2] : null, null, null, null, pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                null, null, null, null, null, null, null, null, null, null, text);
    }

    private void close(Limelight3A limelight) {
        log.close();
        limelight.stop();
    }
}
