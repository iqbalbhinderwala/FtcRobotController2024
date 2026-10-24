package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Auto.AutoSettings;
import org.firstinspires.ftc.teamcode.John.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;
import org.firstinspires.ftc.teamcode.John.Vision.CellTracker;
import org.firstinspires.ftc.teamcode.John.Vision.HiveCamera;
import org.firstinspires.ftc.teamcode.John.Vision.HiveTags;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Calibrates the HIVE tip detector (CellTracker.tipping: the up CELL's tags measured TIP_DROP_IN
 * lower than at the start of a volley, in TIP_FRAMES frames in a row). The robot does not drive;
 * place it where it will shoot from (or at the autonomous' look spot), camera facing the HIVE. It
 * uses the real CellTracker and Shooter, with the values live from the Dashboard (JohnCellTracker).
 *
 * What it measures:
 *  - Heights: each CELL's tag height (mean, noise as a standard deviation over the last frames) and
 *    whether the camera sees it, against HiveTags' nominal 49.9" up / 35.6" down. More than
 *    HEIGHT_WARN_IN off: check the camera mount (JohnCamera pitch / height) or the tag size.
 *  - TIP trial (A): arm with the up CELL's height now as the baseline (as a volley does), then tip
 *    the HIVE (by hand, or with RB shooting into it). It reports when the drop started (first frame
 *    MOTION_IN lower), when the detector fired and the delay, and the lowest height reached.
 *  - NOISE trial (RB while not armed): fires a volley (Shooter, at the flywheel's current target) at
 *    a CELL that will NOT tip (hold the HIVE, or aim beside it), armed: any trigger is a false
 *    alarm. It also counts the frames in which flying balls hid the up CELL's tags.
 * Suggestions (telemetry and the log): TIP_DROP_IN at least 4 x the noise seen during volleys (and at
 * most a third of the up / down difference); TIP_FRAMES up if there were false alarms.
 *
 * INIT: B red, X blue HIVE (default from Game Setup). gamepad1:
 *   A        arm a TIP trial / disarm
 *   RB       fire a volley (armed: part of the TIP trial; not armed: a NOISE trial)
 *   Y        flywheel on/off;  dpad up / down  flywheel target + / - 100 RPM
 *   dpad left  forget the trials;  B  stop the shooter
 * The up CELL follows the camera (CellTracker); the TeleOp / Auto use the same.
 *
 * Logged to /sdcard/FIRST/john_logs/r<run>_john_tip_cal_<time>.csv (one row per camera frame and
 * an event row per trial).
 */
@Config("JohnTipCalibration")
@TeleOp(name = "[John] Tip Detector Calibration", group = "JohnTest")
public class TipCalibration extends LinearOpMode {

    /** Frames for the noise / mean of each CELL's height. */
    public static int WINDOW = 30;
    /** A drop this large marks the start of the tip (for the delay). */
    public static double MOTION_IN = 0.75;
    /** Warn when a measured CELL height is this far from the nominal one. */
    public static double HEIGHT_WARN_IN = 2.0;

    private static final String LOG_HEADER = "t_s,trial,armed,firing,up_cell,baseline,aud_tags,aud_h,far_tags,far_h,"
            + "up_h,drop,low_frames,triggered,rpm,event";

    private static final class Trial {
        String kind;
        double startS;
        double baseline;
        double motionS = Double.NaN;
        double triggerS = Double.NaN;
        double lowest = Double.NaN;
        int volleyFrames;
        int hiddenFrames;
    }

    private final ArrayDeque<Double> audHeights = new ArrayDeque<>();
    private final ArrayDeque<Double> farHeights = new ArrayDeque<>();
    private final List<Double> volleyNoise = new ArrayList<>();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Follower follower = Constants.create(hardwareMap);   // only for "standing still"; it never drives
        HiveCamera camera = new HiveCamera(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        boolean red = new AutoSettings(blackboard).alliance() == AutoSettings.Alliance.RED;
        CsvLog log = new CsvLog("john_tip_cal", LOG_HEADER);

        gamepad1.resetEdgeDetection();
        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) red = true;
            if (gamepad1.xWasPressed()) red = false;
            telemetry.addData("HIVE", "%s   (B red, X blue)", red ? "RED" : "BLUE");
            telemetry.addLine("The robot does not drive. Place it where it shoots from, camera facing the HIVE.");
            telemetry.addData("Camera", camera.status());
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();
        }
        if (isStopRequested()) {
            camera.stop();
            log.close();
            return;
        }

        CellTracker tracker = new CellTracker(red);
        gamepad1.resetEdgeDetection();
        ElapsedTime clock = new ElapsedTime();
        List<Trial> trials = new ArrayList<>();
        Trial trial = null;
        int falseAlarms = 0;
        int tipsSeen = 0;

        while (opModeIsActive()) {
            String event = null;
            boolean arm = gamepad1.aWasPressed(), fire = gamepad1.rightBumperWasPressed();
            boolean fly = gamepad1.yWasPressed(), faster = gamepad1.dpadUpWasPressed(), slower = gamepad1.dpadDownWasPressed();
            boolean forget = gamepad1.dpadLeftWasPressed(), stop = gamepad1.bWasPressed();
            double now = clock.seconds();

            follower.update();
            HiveCamera.Frame frame = camera.poll();
            boolean used = tracker.update(frame, follower);
            if (tracker.event() != null) {
                event = tracker.event();
            }

            // ---- Heights of both CELLs of our HIVE in this frame ----
            int audTags = 0, farTags = 0;
            double audSum = 0, farSum = 0;
            if (used) {
                for (HiveCamera.Tag t : frame.tags) {
                    if (HiveTags.red(t.id) != red) continue;
                    if (HiveTags.audience(t.id)) {
                        audTags++;
                        audSum += t.up;
                    } else {
                        farTags++;
                        farSum += t.up;
                    }
                }
                if (audTags > 0) push(audHeights, audSum / audTags);
                if (farTags > 0) push(farHeights, farSum / farTags);
            }
            double audH = audTags > 0 ? audSum / audTags : Double.NaN;
            double farH = farTags > 0 ? farSum / farTags : Double.NaN;
            double upH = tracker.upAudience() ? audH : farH;
            ArrayDeque<Double> upHistory = tracker.upAudience() ? audHeights : farHeights;

            // ---- Buttons ----
            if (arm) {
                if (trial != null) {
                    // A TIP trial that never fired counts as a missed tip
                    if (trial.kind.equals("TIP") && Double.isNaN(trial.triggerS)) {
                        trials.add(trial);
                        event = "disarmed: " + describe(trial);
                    } else {
                        event = "disarmed";
                    }
                    shooter.cancel();
                    tracker.tipping(Double.NaN);
                    trial = null;
                } else {
                    trial = startTrial("TIP", mean(upHistory, 10), now);
                    event = trial == null ? "cannot arm: the camera does not see the up CELL" : String.format(Locale.US,
                            "TIP trial armed: baseline %.1f in (%s CELL); now tip the HIVE", trialBaseline(trial),
                            tracker.upCellName());
                }
            }
            if (fire && !shooter.firing()) {
                if (trial == null) {
                    trial = startTrial("NOISE", mean(upHistory, 10), now);
                    if (trial == null) {
                        event = "cannot start a NOISE trial: the camera does not see the up CELL";
                    }
                }
                if (trial != null) {
                    shooter.startVolley();
                    event = (event == null ? "" : event + "; ") + trial.kind + " trial: volley";
                }
            }
            if (fly) {
                shooter.setSpin(!shooter.flywheel().isOn());
            }
            if (faster) shooter.flywheel().setTargetRpm(shooter.flywheel().targetRpm() + 100);
            if (slower) shooter.flywheel().setTargetRpm(shooter.flywheel().targetRpm() - 100);
            if (stop) {
                shooter.cancel();
                shooter.setSpin(false);
                event = "shooter stopped";
            }
            if (forget) {
                trials.clear();
                volleyNoise.clear();
                falseAlarms = 0;
                tipsSeen = 0;
                event = "trials forgotten";
            }

            // ---- The detector, exactly as a volley runs it ----
            boolean triggered = false;
            if (trial != null) {
                triggered = tracker.tipping(trial.baseline);
                if (shooter.firing()) {
                    trial.volleyFrames += frame != null ? 1 : 0;
                    trial.hiddenFrames += frame != null && Double.isNaN(upH) ? 1 : 0;
                    if (!Double.isNaN(upH) && trial.kind.equals("NOISE")) {
                        volleyNoise.add(upH - trial.baseline);
                    }
                }
                if (!Double.isNaN(upH)) {
                    trial.lowest = Double.isNaN(trial.lowest) ? upH : Math.min(trial.lowest, upH);
                    if (Double.isNaN(trial.motionS) && upH < trial.baseline - MOTION_IN) {
                        trial.motionS = now;
                    }
                }
                if (triggered && Double.isNaN(trial.triggerS)) {
                    trial.triggerS = now;
                    shooter.cancel();   // as a volley does
                    if (trial.kind.equals("TIP")) {
                        tipsSeen++;
                        event = String.format(Locale.US, "TIP detected %.2f s after the drop started (%.1f in below the baseline)",
                                Double.isNaN(trial.motionS) ? Double.NaN : now - trial.motionS, trial.baseline - upH);
                    } else {
                        falseAlarms++;
                        event = String.format(Locale.US, "FALSE ALARM during the NOISE volley (%.1f in below the baseline)",
                                trial.baseline - upH);
                    }
                }
                boolean over = trial.kind.equals("NOISE") ? !shooter.firing()
                        : !Double.isNaN(trial.triggerS) && now - trial.triggerS > 1.5;
                if (over) {
                    trials.add(trial);
                    event = (event == null ? "" : event + "; ") + describe(trial);
                    tracker.tipping(Double.NaN);
                    trial = null;
                }
            }

            String shooterEvent = shooter.update();
            if (shooterEvent != null) {
                event = event == null ? shooterEvent : event + "; " + shooterEvent;
            }

            // ---- Telemetry ----
            telemetry.addData("Up CELL (camera)", "%s; confirmed %.1f s ago", tracker.upCellName(),
                    tracker.msSinceConfirmed() / 1000.0);
            telemetry.addData("Audience CELL", cellLine(audHeights, audTags, HiveTags.cellZ(tracker.upAudience())));
            telemetry.addData("Far CELL", cellLine(farHeights, farTags, HiveTags.cellZ(!tracker.upAudience())));
            telemetry.addData("Detector", "TIP_DROP_IN %.1f, TIP_FRAMES %d%s", CellTracker.TIP_DROP_IN,
                    CellTracker.TIP_FRAMES, trial == null ? "" : String.format(Locale.US,
                            ";  %s trial: baseline %.1f, now %s, low frames %d%s", trial.kind, trial.baseline,
                            Double.isNaN(upH) ? "not seen" : String.format(Locale.US, "%.1f (drop %.1f)", upH, trial.baseline - upH),
                            tracker.lowFrames(), Double.isNaN(trial.triggerS) ? "" : "  TRIGGERED"));
            telemetry.addData("Shooter", shooter.status());
            telemetry.addData("Results", "%d tips detected, %d false alarms in %d trials", tipsSeen, falseAlarms, trials.size());
            for (int i = Math.max(0, trials.size() - 4); i < trials.size(); i++) {
                telemetry.addLine("  " + describe(trials.get(i)));
            }
            telemetry.addData("Suggestion", suggestion(falseAlarms));
            telemetry.addLine("A arm TIP trial, RB volley (NOISE trial if not armed), Y flywheel, dpad up/down RPM, "
                    + "dpad left forget, B stop shooter");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            if (!Double.isNaN(audH)) packet.put("audienceCellIn", audH);
            if (!Double.isNaN(farH)) packet.put("farCellIn", farH);
            if (trial != null) packet.put("baselineIn", trial.baseline);
            dashboard.sendTelemetryPacket(packet);

            if (frame != null || event != null) {
                log.row(now, trial == null ? null : trial.kind, trial != null ? 1 : 0, shooter.firing() ? 1 : 0,
                        tracker.upCellName(), trial == null ? null : trial.baseline, audTags, nan(audH), farTags, nan(farH),
                        nan(upH), trial == null || Double.isNaN(upH) ? null : trial.baseline - upH, tracker.lowFrames(),
                        triggered ? 1 : 0, shooter.flywheel().rpm(), event);
            }
        }
        log.row(clock.seconds(), null, null, null, null, null, null, null, null, null, null, null, null, null, null,
                "summary: " + tipsSeen + " tips detected, " + falseAlarms + " false alarms; " + suggestion(falseAlarms));
        shooter.stop();
        camera.stop();
        log.close();
    }

    private static Trial startTrial(String kind, double baseline, double now) {
        if (Double.isNaN(baseline)) {
            return null;
        }
        Trial t = new Trial();
        t.kind = kind;
        t.startS = now;
        t.baseline = baseline;
        return t;
    }

    private static double trialBaseline(Trial t) {
        return t == null ? Double.NaN : t.baseline;
    }

    private static String describe(Trial t) {
        if (t.kind.equals("TIP")) {
            return String.format(Locale.US, "TIP from %.1f: %s, lowest %.1f%s", t.baseline,
                    Double.isNaN(t.triggerS) ? "NOT detected"
                            : Double.isNaN(t.motionS) ? "detected" : String.format(Locale.US, "detected %.2f s after the drop began",
                                    t.triggerS - t.motionS),
                    t.lowest, hidden(t));
        }
        return String.format(Locale.US, "NOISE volley at %.1f: %s, lowest %.1f%s", t.baseline,
                Double.isNaN(t.triggerS) ? "no false alarm" : "FALSE ALARM", t.lowest, hidden(t));
    }

    private static String hidden(Trial t) {
        return t.volleyFrames == 0 ? "" : String.format(Locale.US, ", up CELL hidden in %d / %d volley frames",
                t.hiddenFrames, t.volleyFrames);
    }

    /** Noise during the NOISE volleys, and the TIP_DROP_IN / TIP_FRAMES that fits it. */
    private String suggestion(int falseAlarms) {
        if (volleyNoise.size() < 10) {
            return "run a few NOISE trials (RB, HIVE held) and TIP trials (A, then tip it)";
        }
        double sd = sd(volleyNoise);
        double swing = HiveTags.cellZ(true) - HiveTags.cellZ(false);
        double drop = Math.min(swing / 3.0, Math.max(1.0, 4.0 * sd));
        return String.format(Locale.US, "height noise during volleys %.2f in: TIP_DROP_IN about %.1f%s", sd, drop,
                falseAlarms > 0 ? String.format(Locale.US, ", TIP_FRAMES %d (false alarms)", CellTracker.TIP_FRAMES + 1) : "");
    }

    private String cellLine(ArrayDeque<Double> history, int tagsNow, double nominal) {
        if (history.isEmpty()) {
            return "never seen";
        }
        double m = mean(history, history.size());
        return String.format(Locale.US, "%s, mean %.1f in (+/- %.2f over %d frames), nominal %.1f%s",
                tagsNow > 0 ? tagsNow + " tags now" : "not seen now", m, sd(new ArrayList<>(history)), history.size(), nominal,
                Math.abs(m - nominal) > HEIGHT_WARN_IN ? "  <- OFF: check JohnCamera pitch / height, tag size" : "");
    }

    private static void push(ArrayDeque<Double> d, double v) {
        d.addLast(v);
        while (d.size() > Math.max(2, WINDOW)) {
            d.removeFirst();
        }
    }

    /** Mean of the last {@code n} values, or NaN. */
    private static double mean(ArrayDeque<Double> d, int n) {
        if (d.isEmpty()) {
            return Double.NaN;
        }
        List<Double> v = new ArrayList<>(d);
        double s = 0;
        int k = 0;
        for (int i = Math.max(0, v.size() - n); i < v.size(); i++) {
            s += v.get(i);
            k++;
        }
        return s / k;
    }

    private static double sd(List<Double> v) {
        if (v.size() < 2) {
            return 0.0;
        }
        double m = 0;
        for (double x : v) m += x;
        m /= v.size();
        double s = 0;
        for (double x : v) s += (x - m) * (x - m);
        return Math.sqrt(s / (v.size() - 1));
    }

    private static Double nan(double v) {
        return Double.isNaN(v) ? null : v;
    }
}
