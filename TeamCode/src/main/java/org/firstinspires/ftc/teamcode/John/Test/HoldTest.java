package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.JohnGains;
import org.firstinspires.ftc.teamcode.John.Pedro.JohnTuning;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

import java.util.ArrayDeque;
import java.util.Arrays;

/**
 * Hold test for tuning John's heading controller live from FTC Dashboard.
 *
 * Poses are FTC field coordinates (origin at the field center, +X toward the audience, +Y toward
 * blue, inches). The robot is told it starts at START_X_IN / START_Y_IN / START_HEADING_DEG; that only
 * places it on the Dashboard field, the hold itself is relative.
 *
 * START holds the current pose with Pedro. Edit any JohnTuning value (heading, forward, strafe) on the
 * Dashboard: the controller is rebuilt and swapped in right away, while the robot keeps holding.
 * gamepad1 (step tests): left / right bumper turn the held heading left / right by STEP_DEG, dpad
 * up / down move the held point forward / back and dpad left / right strafe it left / right by
 * STEP_IN, A holds where the robot is now.
 *
 * Telemetry and the Dashboard graph show the heading / position error, plus the heading swing
 * (peak to peak) and its frequency over the last WINDOW_S: a steady swing that does not die down is
 * oscillation. OdometryWatchdog pauses the drive on bad Pinpoint reads and resumes by itself once they
 * are good again; if they stay bad (reboot, dead link) it stops for good (restart the OpMode). Logged to /sdcard/FIRST/john_logs/r<run>_john_hold_test_<time>.csv.
 */
@Config
@TeleOp(name = "[John] Hold Test", group = "JohnTest")
public class HoldTest extends LinearOpMode {

    public static double STEP_DEG = 15.0;
    public static double STEP_IN = 6.0;
    public static double WINDOW_S = 2.0;
    /** Start pose, FTC field coordinates (default: a tile center on the blue side, toward the audience). */
    public static double START_X_IN = 36.0;
    public static double START_Y_IN = 36.0;
    public static double START_HEADING_DEG = 180.0;

    private static final String LOG_HEADER = "t_s,target_x,target_y,target_h_deg,x,y,h_deg,"
            + "heading_err_deg,position_err_in,swing_pp_deg,swing_hz,loop_ms,kp,brake_a,brake_b,ks,ks_deadband_deg,watchdog,pinpoint_status,pinpoint_loop_us,"
            + "p_fl,p_fr,p_bl,p_br,event";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CsvLog log = new CsvLog("john_hold_test", LOG_HEADER);

        Follower follower = Constants.create(hardwareMap);
        OdometryWatchdog watchdog = new OdometryWatchdog(hardwareMap, "hold_test");
        // Drive motor powers as commanded (for spotting wheels hunting back and forth near the target)
        DcMotor[] motors = {
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get())};
        follower.setPose(new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG)));
        watchdog.poseSet();

        telemetry.addData("Status", "Press START to hold. Tune JohnTuning on the Dashboard.");
        telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
        telemetry.update();
        waitForStart();
        gamepad1.resetEdgeDetection();   // ignore presses made during INIT

        ElapsedTime clock = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        follower.update();
        Pose target = follower.pose();
        follower.hold(target);
        double[] applied = tuning();
        double[] appliedAll = JohnGains.current().toArray();
        log.row(clock.seconds(), null, null, null, null, null, null, null, null, null, null, null,
                applied[0], applied[1], applied[2], applied[3], applied[4], watchdog.state(), watchdog.status(), watchdog.pinpointLoopUs(),
                motors[0].getPower(), motors[1].getPower(), motors[2].getPower(), motors[3].getPower(), "start holding: " + JohnGains.current().summary());

        ArrayDeque<double[]> history = new ArrayDeque<>();   // {t, heading error deg}

        while (opModeIsActive()) {
            // New tuning values from the Dashboard: rebuild the controller, keep holding the same target
            double[] nowAll = JohnGains.current().toArray();
            if (watchdog.ok() && !Arrays.equals(nowAll, appliedAll)) {
                double[] now = tuning();
                follower.setAlgorithm(new Foresight(Constants.foresightConfig()));
                follower.hold(target);
                applied = now;
                appliedAll = nowAll;
                log.row(clock.seconds(), null, null, null, null, null, null, null, null, null, null, null,
                        applied[0], applied[1], applied[2], applied[3], applied[4], watchdog.state(), watchdog.status(),
                        watchdog.pinpointLoopUs(),
                        motors[0].getPower(), motors[1].getPower(), motors[2].getPower(), motors[3].getPower(), "new tuning applied: " + JohnGains.current().summary());
            }

            // Step tests (one per press; every button is read each loop)
            boolean left = gamepad1.leftBumperWasPressed(), right = gamepad1.rightBumperWasPressed();
            boolean up = gamepad1.dpadUpWasPressed(), down = gamepad1.dpadDownWasPressed(), a = gamepad1.aWasPressed();
            boolean strafeLeft = gamepad1.dpadLeftWasPressed(), strafeRight = gamepad1.dpadRightWasPressed();
            Pose newTarget = null;
            String step = null;
            if (left) {
                newTarget = target.withHeading(target.heading() + Math.toRadians(STEP_DEG));
                step = "turn +" + STEP_DEG;
            } else if (right) {
                newTarget = target.withHeading(target.heading() - Math.toRadians(STEP_DEG));
                step = "turn -" + STEP_DEG;
            } else if (up) {
                newTarget = new Pose(target.x() + STEP_IN * Math.cos(target.heading()),
                        target.y() + STEP_IN * Math.sin(target.heading()), target.heading());
                step = "forward " + STEP_IN;
            } else if (down) {
                newTarget = new Pose(target.x() - STEP_IN * Math.cos(target.heading()),
                        target.y() - STEP_IN * Math.sin(target.heading()), target.heading());
                step = "back " + STEP_IN;
            } else if (strafeLeft) {
                // Left of the heading: +90 deg
                newTarget = new Pose(target.x() - STEP_IN * Math.sin(target.heading()),
                        target.y() + STEP_IN * Math.cos(target.heading()), target.heading());
                step = "strafe left " + STEP_IN;
            } else if (strafeRight) {
                newTarget = new Pose(target.x() + STEP_IN * Math.sin(target.heading()),
                        target.y() - STEP_IN * Math.cos(target.heading()), target.heading());
                step = "strafe right " + STEP_IN;
            } else if (a) {
                newTarget = follower.pose();
                step = "hold here";
            }
            if (newTarget != null && watchdog.ok()) {
                target = newTarget;
                follower.hold(target);
                log.row(clock.seconds(), null, null, null, null, null, null, null, null, null, null, null,
                        applied[0], applied[1], applied[2], applied[3], applied[4], watchdog.state(), watchdog.status(),
                        watchdog.pinpointLoopUs(),
                        motors[0].getPower(), motors[1].getPower(), motors[2].getPower(), motors[3].getPower(), step);
            }

            follower.update();
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            Pose pose = follower.pose();
            double headingErr = Math.toDegrees(normalize(target.heading() - pose.heading()));
            double positionErr = Math.hypot(target.x() - pose.x(), target.y() - pose.y());

            OdometryWatchdog.State before = watchdog.state();
            OdometryWatchdog.State after = watchdog.check(follower);
            if (after != before) {
                String change;
                if (after == OdometryWatchdog.State.PAUSED) {
                    change = "ODOMETRY PAUSED: " + watchdog.reason();
                    gamepad1.rumble(300);
                } else if (after == OdometryWatchdog.State.OK) {
                    change = String.format("ODOMETRY RESUMED after %.0f ms", watchdog.lastPauseMs());
                } else {
                    change = "ODOMETRY LOST: " + watchdog.reason();
                    gamepad1.rumble(1000);
                }
                log.row(clock.seconds(), null, null, null, null, null, null, null, null, null, null, null,
                        applied[0], applied[1], applied[2], applied[3], applied[4], watchdog.state(), watchdog.status(),
                        watchdog.pinpointLoopUs(),
                        motors[0].getPower(), motors[1].getPower(), motors[2].getPower(), motors[3].getPower(), change);
            }

            // Heading swing over the last WINDOW_S: peak to peak and zero-crossing frequency
            double t = clock.seconds();
            history.addLast(new double[]{t, headingErr});
            while (!history.isEmpty() && history.peekFirst()[0] < t - WINDOW_S) {
                history.removeFirst();
            }
            double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY, mean = 0;
            for (double[] h : history) {
                min = Math.min(min, h[1]);
                max = Math.max(max, h[1]);
                mean += h[1];
            }
            mean /= history.size();
            int crossings = 0;
            Double previous = null;
            for (double[] h : history) {
                double centered = h[1] - mean;
                if (previous != null && Math.signum(centered) != Math.signum(previous) && centered != 0) {
                    crossings++;
                }
                previous = centered;
            }
            double swing = max - min;
            double hz = crossings / 2.0 / WINDOW_S;

            if (watchdog.lost()) {
                telemetry.addLine("ODOMETRY LOST, drive stopped: " + watchdog.reason());
                telemetry.addLine("Check the Pinpoint cable / LED, then restart the OpMode.");
            } else if (!watchdog.ok()) {
                telemetry.addLine("ODOMETRY PAUSED: " + watchdog.reason());
            }
            telemetry.addData("Pinpoint", "%s, loop %d us, %d glitch pause(s), last %.0f ms",
                    watchdog.status(), watchdog.pinpointLoopUs(), watchdog.pauses(), watchdog.lastPauseMs());
            telemetry.addData("Pose (FTC)", Drawing.format(pose));
            telemetry.addData("Heading error", "%+.2f deg", headingErr);
            telemetry.addData("Position error", "%.2f in", positionErr);
            telemetry.addData("Heading swing", "%.2f deg peak-to-peak, ~%.1f Hz (last %.0f s)", swing, hz, WINDOW_S);
            telemetry.addData("Forward / strafe", "P small %.3f / %.3f, KS %.3f / %.3f",
                    JohnTuning.FORWARD_P_SMALL, JohnTuning.STRAFE_P_SMALL, JohnTuning.FORWARD_KS, JohnTuning.STRAFE_KS);
            telemetry.addData("Tuning", "KP %.3f  KS %.3f (deadband %.2f deg)  brake A %.4f  B %.5f",
                    applied[0], applied[3], applied[4], applied[1], applied[2]);
            telemetry.addData("Loop", "%.1f ms", loopMs);
            telemetry.addData("Controls", "bumpers turn %.0f deg, dpad move / strafe %.0f in, A hold here",
                    STEP_DEG, STEP_IN);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            packet.put("headingErrorDeg", headingErr);
            packet.put("positionErrorIn", positionErr);
            packet.put("headingSwingDeg", swing);
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobotOutline(packet.fieldOverlay(), target, "lime", "target");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            dashboard.sendTelemetryPacket(packet);

            log.row(t, target.x(), target.y(), Math.toDegrees(target.heading()), pose.x(), pose.y(),
                    Math.toDegrees(pose.heading()), headingErr, positionErr, swing, hz, loopMs,
                    applied[0], applied[1], applied[2], applied[3], applied[4], watchdog.state(), watchdog.status(),
                    watchdog.pinpointLoopUs(),
                        motors[0].getPower(), motors[1].getPower(), motors[2].getPower(), motors[3].getPower(), null);
        }
        log.close();
        watchdog.close();
    }

    private static double[] tuning() {
        return new double[]{JohnTuning.HEADING_KP, JohnTuning.HEADING_BRAKE_A, JohnTuning.HEADING_BRAKE_B,
                JohnTuning.HEADING_KS, JohnTuning.HEADING_KS_DEADBAND_DEG};
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
