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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.JohnGains;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Automatic sweep of John's forward and strafe position controllers over moves of different
 * lengths (and so different speeds). The heading controller uses the current JohnTuning values.
 *
 * Forward and strafe are separate controllers, so they are swept separately (AXES). For the
 * forward axis every combination of FORWARD_P_SMALL_VALUES x FORWARD_KS_VALUES is tried, for the
 * strafe axis STRAFE_P_SMALL_VALUES x STRAFE_KS_VALUES (comma-separated lists, editable on the
 * Dashboard before START). For each combination the robot moves each of DISTANCES_IN, first
 * forward (or left) then back (or right), so it ends where it started, REPEATS times. Each move is
 * watched for MOVE_WINDOW_S and scored:
 *   cost = time to settle within SETTLE_TOL_IN (+ UNSETTLED_PENALTY_S if it never does)
 *          + W_FINAL x final error + W_OVERSHOOT x overshoot + W_SWING x largest swing (inches)
 *          + W_CHATTER x sign flips per second of that axis's motor command while resting.
 * At the end the combinations of each axis are ranked by mean cost. X applies the best of each
 * axis to JohnTuning (until the app restarts) and holds with them. B aborts and ranks what has
 * been done so far.
 *
 * The robot needs room for the longest distance in front / behind and to the sides.
 * OdometryWatchdog pauses the drive on bad Pinpoint reads; moves that saw a pause are not scored,
 * and a LOST odometry aborts the sweep.
 *
 * Logs (/sdcard/FIRST/john_logs/), sharing one run number: r<run>_john_drive_cal_samples (every
 * loop), _moves (one row per move), _ranking (per combination and per combination x distance).
 */
@Config
@TeleOp(name = "[John] Drive Calibration", group = "JohnTest")
public class DriveCalibration extends LinearOpMode {

    // ---- Sweep (comma-separated lists) ----
    /** "forward", "strafe" or "forward,strafe". */
    public static String AXES = "forward,strafe";
    public static String FORWARD_P_SMALL_VALUES = "0.152";
    public static String FORWARD_KS_VALUES = "0,0.02,0.04,0.06";
    public static String STRAFE_P_SMALL_VALUES = "0.25";
    public static String STRAFE_KS_VALUES = "0,0.05,0.1,0.15";
    public static String DISTANCES_IN = "3,6,12,24";
    public static int REPEATS = 2;

    // ---- Timing ----
    public static double MOVE_WINDOW_S = 1.8;
    public static double COMBO_SETTLE_S = 1.0;
    /** Resting part of a move used for the chatter count: its last this many seconds. */
    public static double REST_S = 0.5;

    // ---- Scoring ----
    public static double SETTLE_TOL_IN = 0.25;
    public static double UNSETTLED_PENALTY_S = 0.5;
    public static double W_FINAL = 1.0;
    public static double W_OVERSHOOT = 0.5;
    public static double W_SWING = 0.5;
    public static double W_CHATTER = 0.05;
    /** Smallest move counted as a swing (filters sensor noise), inches. */
    public static double SWING_HYSTERESIS_IN = 0.05;
    /** Command sign flips smaller than this are ignored. */
    public static double CHATTER_MIN_POWER = 0.01;

    /** Start pose, FTC field coordinates (only places the robot on the Dashboard field). */
    public static double START_X_IN = 36.0;
    public static double START_Y_IN = 36.0;
    public static double START_HEADING_DEG = 180.0;

    private static final String SAMPLE_HEADER = "t_s,axis,combo,move,distance_in,target_x,target_y,x,y,h_deg,"
            + "along_err_in,cross_err_in,heading_err_deg,watchdog,cmd_forward,cmd_strafe_left,cmd_turn";
    private static final String MOVE_HEADER = "t_s,axis,combo,move,p_small,ks,distance_in,valid,t90_s,settle_s,"
            + "settled,overshoot_in,first_stop_in,max_swing_in,swings,final_in,final_along_in,max_cross_in,"
            + "max_heading_err_deg,chatter_per_s,rest_command,peak_speed_ips,cost,voltage";
    private static final String RANKING_HEADER = "axis,rank,p_small,ks,distance_in,moves,cost,t90_s,settle_s,"
            + "settled_frac,overshoot_in,max_swing_in,final_in,chatter_per_s";

    private Follower follower;
    private OdometryWatchdog watchdog;
    private DcMotor[] motors;
    private FtcDashboard dashboard;
    private CsvLog samples;
    private final ElapsedTime clock = new ElapsedTime();
    private Pose target;
    private boolean aborted;
    private int comboIndex;
    private int moveIndex;
    private double currentDistance;

    private static class Combo {
        final boolean strafe;
        final double pSmall, ks;
        final List<MoveResult> moves = new ArrayList<>();

        Combo(boolean strafe, double pSmall, double ks) {
            this.strafe = strafe;
            this.pSmall = pSmall;
            this.ks = ks;
        }

        String label() {
            return String.format(Locale.US, "%s P %.3f KS %.3f", strafe ? "strafe" : "forward", pSmall, ks);
        }
    }

    /** Metrics of one move. Along-errors are signed toward the move: + = short, - = past. */
    private static class MoveResult {
        double distance;
        boolean valid = true;
        double t90 = Double.NaN;
        double settle;
        boolean settled;
        double overshoot, firstStop, maxSwing, finalErr, finalAlong, maxCross, maxHeadingErr;
        int swings;
        double chatter, restCommand, peakSpeed, cost;
    }

    private static class Stats {
        int n;
        double cost, t90, settle, settledFrac, overshoot, maxSwing, finalErr, chatter;

        static Stats of(List<MoveResult> moves, double distanceOrNaN) {
            Stats s = new Stats();
            int n90 = 0;
            for (MoveResult r : moves) {
                if (!r.valid || (!Double.isNaN(distanceOrNaN) && r.distance != distanceOrNaN)) {
                    continue;
                }
                s.n++;
                s.cost += r.cost;
                s.settle += r.settle;
                s.settledFrac += r.settled ? 1 : 0;
                s.overshoot += r.overshoot;
                s.maxSwing += r.maxSwing;
                s.finalErr += r.finalErr;
                s.chatter += r.chatter;
                if (!Double.isNaN(r.t90)) {
                    s.t90 += r.t90;
                    n90++;
                }
            }
            if (s.n > 0) {
                s.cost /= s.n;
                s.settle /= s.n;
                s.settledFrac /= s.n;
                s.overshoot /= s.n;
                s.maxSwing /= s.n;
                s.finalErr /= s.n;
                s.chatter /= s.n;
            }
            s.t90 = n90 > 0 ? s.t90 / n90 : Double.NaN;
            return s;
        }
    }

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        boolean doForward = AXES.toLowerCase(Locale.US).contains("forward");
        boolean doStrafe = AXES.toLowerCase(Locale.US).contains("strafe");
        double[] fps = parse(FORWARD_P_SMALL_VALUES);
        double[] fks = parse(FORWARD_KS_VALUES);
        double[] sps = parse(STRAFE_P_SMALL_VALUES);
        double[] sks = parse(STRAFE_KS_VALUES);
        double[] distances = parse(DISTANCES_IN);
        List<Combo> combos = new ArrayList<>();
        if (doForward) {
            for (double p : fps) {
                for (double ks : fks) {
                    combos.add(new Combo(false, p, ks));
                }
            }
        }
        if (doStrafe) {
            for (double p : sps) {
                for (double ks : sks) {
                    combos.add(new Combo(true, p, ks));
                }
            }
        }
        int movesPerCombo = distances.length * 2 * Math.max(1, REPEATS);
        double estimateS = combos.size() * (COMBO_SETTLE_S + movesPerCombo * MOVE_WINDOW_S);
        double maxDistance = 0;
        for (double d : distances) {
            maxDistance = Math.max(maxDistance, d);
        }

        follower = Constants.create(hardwareMap);
        watchdog = new OdometryWatchdog(hardwareMap);
        follower.setPose(new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG)));
        watchdog.poseSet();
        motors = new DcMotor[]{
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get())};

        int run = CsvLog.nextRunNumber();
        samples = new CsvLog("john_drive_cal_samples", SAMPLE_HEADER, run);
        CsvLog moveLog = new CsvLog("john_drive_cal_moves", MOVE_HEADER, run);
        CsvLog rankingLog = new CsvLog("john_drive_cal_ranking", RANKING_HEADER, run);

        boolean listsOk = !combos.isEmpty() && distances.length > 0;
        telemetry.addLine(listsOk ? String.format(Locale.US,
                "Drive calibration: needs %.0f\" of room in front, behind%s.", maxDistance,
                doStrafe ? " and to both sides" : "")
                : "ERROR: no combinations (check AXES and the value lists).");
        telemetry.addData("Combinations", "%d x %d moves (distances %s, there and back, x%d)",
                combos.size(), movesPerCombo, DISTANCES_IN, Math.max(1, REPEATS));
        telemetry.addData("Estimated time", "%.0f s", estimateS);
        telemetry.addData("Heading", JohnGains.current().summary());
        telemetry.addData("Log", samples.error().isEmpty() ? samples.fileName() : "ERROR " + samples.error());
        telemetry.addLine("START runs the sweep. B aborts.");
        telemetry.update();
        waitForStart();
        if (!listsOk) {
            return;
        }

        clock.reset();
        follower.update();
        target = follower.pose();
        follower.hold(target);

        for (comboIndex = 0; comboIndex < combos.size() && !aborted; comboIndex++) {
            Combo combo = combos.get(comboIndex);
            JohnGains gains = JohnGains.current();
            if (combo.strafe) {
                gains.strafePSmall = combo.pSmall;
                gains.strafeKs = combo.ks;
            } else {
                gains.forwardPSmall = combo.pSmall;
                gains.forwardKs = combo.ks;
            }
            follower.setAlgorithm(new Foresight(Constants.foresightConfig(gains)));
            follower.hold(target);
            currentDistance = 0;
            run(COMBO_SETTLE_S, null, combo, combos.size(), 0, 0);

            moveIndex = 0;
            for (int rep = 0; rep < Math.max(1, REPEATS) && !aborted; rep++) {
                for (double distance : distances) {
                    for (int dir : new int[]{1, -1}) {
                        if (aborted) {
                            break;
                        }
                        currentDistance = dir * distance;
                        // Unit vector of the move in the field: forward along the heading, strafe to its left
                        double h = target.heading() + (combo.strafe ? Math.PI / 2.0 : 0.0);
                        double ux = dir * Math.cos(h);
                        double uy = dir * Math.sin(h);
                        target = new Pose(target.x() + distance * ux, target.y() + distance * uy, target.heading());
                        follower.hold(target);
                        MoveResult r = move(combo, combos.size(), distance, ux, uy);
                        combo.moves.add(r);
                        moveLog.row(clock.seconds(), combo.strafe ? "strafe" : "forward", comboIndex, moveIndex,
                                combo.pSmall, combo.ks, currentDistance, r.valid ? 1 : 0, r.t90, r.settle,
                                r.settled ? 1 : 0, r.overshoot, r.firstStop, r.maxSwing, r.swings, r.finalErr,
                                r.finalAlong, r.maxCross, r.maxHeadingErr, r.chatter, r.restCommand, r.peakSpeed,
                                r.cost, voltage());
                        moveIndex++;
                    }
                }
            }
        }

        // Ranking per axis
        List<Combo> forwardRanked = rank(combos, false);
        List<Combo> strafeRanked = rank(combos, true);
        logRanking(rankingLog, forwardRanked, distances);
        logRanking(rankingLog, strafeRanked, distances);
        samples.close();
        moveLog.close();
        rankingLog.close();

        // Results: hold, show the rankings, X applies the best of each axis
        boolean applied = false;
        boolean lastX = false;
        while (opModeIsActive()) {
            boolean x = gamepad1.x;
            if (x && !lastX && !watchdog.lost() && (!forwardRanked.isEmpty() || !strafeRanked.isEmpty())) {
                JohnGains best = JohnGains.current();
                if (!forwardRanked.isEmpty()) {
                    best.forwardPSmall = forwardRanked.get(0).pSmall;
                    best.forwardKs = forwardRanked.get(0).ks;
                }
                if (!strafeRanked.isEmpty()) {
                    best.strafePSmall = strafeRanked.get(0).pSmall;
                    best.strafeKs = strafeRanked.get(0).ks;
                }
                best.applyToTuning();
                follower.setAlgorithm(new Foresight(Constants.foresightConfig()));
                follower.hold(target);
                applied = true;
            }
            lastX = x;
            follower.update();
            watchdog.check(follower);

            telemetry.addLine(aborted ? "Sweep ABORTED: ranking of the moves done so far" : "Sweep done");
            if (watchdog.lost()) {
                telemetry.addLine("ODOMETRY LOST: " + watchdog.reason());
            }
            showRanking("Forward", forwardRanked, distances);
            showRanking("Strafe", strafeRanked, distances);
            telemetry.addLine(applied ? "Best values applied to JohnTuning (until the app restarts)."
                    : "X: apply the best of each axis to JohnTuning (until the app restarts).");
            telemetry.addData("Logs", "john_logs/%s (+ _moves, _ranking)", samples.fileName());
            telemetry.update();
        }
    }

    private static List<Combo> rank(List<Combo> combos, boolean strafe) {
        List<Combo> ranked = new ArrayList<>();
        for (Combo c : combos) {
            if (c.strafe == strafe && Stats.of(c.moves, Double.NaN).n > 0) {
                ranked.add(c);
            }
        }
        Collections.sort(ranked, (a, b) -> Double.compare(Stats.of(a.moves, Double.NaN).cost,
                Stats.of(b.moves, Double.NaN).cost));
        return ranked;
    }

    private static void logRanking(CsvLog log, List<Combo> ranked, double[] distances) {
        for (int i = 0; i < ranked.size(); i++) {
            Combo c = ranked.get(i);
            String axis = c.strafe ? "strafe" : "forward";
            Stats all = Stats.of(c.moves, Double.NaN);
            log.row(axis, i + 1, c.pSmall, c.ks, "all", all.n, all.cost, all.t90, all.settle, all.settledFrac,
                    all.overshoot, all.maxSwing, all.finalErr, all.chatter);
            for (double d : distances) {
                Stats s = Stats.of(c.moves, d);
                log.row(axis, i + 1, c.pSmall, c.ks, d, s.n, s.cost, s.t90, s.settle, s.settledFrac,
                        s.overshoot, s.maxSwing, s.finalErr, s.chatter);
            }
        }
    }

    private void showRanking(String axis, List<Combo> ranked, double[] distances) {
        if (ranked.isEmpty()) {
            return;
        }
        telemetry.addLine(axis + ": rank  cost | settle  final  overshoot  swing (in, s)  chatter/s | P small, KS");
        for (int i = 0; i < Math.min(4, ranked.size()); i++) {
            Combo c = ranked.get(i);
            Stats s = Stats.of(c.moves, Double.NaN);
            telemetry.addLine(String.format(Locale.US, "  %d  %.2f | %.2f s  %.2f  %.2f  %.2f  %.1f | %.3f, %.3f  (%d)",
                    i + 1, s.cost, s.settle, s.finalErr, s.overshoot, s.maxSwing, s.chatter, c.pSmall, c.ks, s.n));
        }
        Combo best = ranked.get(0);
        for (double d : distances) {
            Stats s = Stats.of(best.moves, d);
            telemetry.addLine(String.format(Locale.US, "    best at %.0f\": settle %.2f s, final %.2f\", "
                    + "overshoot %.2f\", 90%% at %.2f s", d, s.settle, s.finalErr, s.overshoot, s.t90));
        }
    }

    /** Runs one move for MOVE_WINDOW_S and scores it. (ux, uy) = unit vector of the move. */
    private MoveResult move(Combo combo, int comboCount, double distance, double ux, double uy) {
        // {t, along error (+ short), cross error, heading error deg, axis command, x, y}
        List<double[]> trace = new ArrayList<>();
        MoveResult r = new MoveResult();
        r.distance = distance;
        boolean ok = run(MOVE_WINDOW_S, trace, combo, comboCount, ux, uy);
        if (!ok || trace.size() < 10) {
            r.valid = false;
            return r;
        }
        score(r, trace, distance);
        return r;
    }

    /**
     * Runs the loop for a while; with a trace, records each sample of the move.
     * @return false if the watchdog paused or lost odometry during it (the move is not scored)
     */
    private boolean run(double seconds, List<double[]> trace, Combo combo, int comboCount, double ux, double uy) {
        ElapsedTime t = new ElapsedTime();
        boolean clean = true;
        while (opModeIsActive() && t.seconds() < seconds) {
            if (gamepad1.b) {
                aborted = true;
                return false;
            }
            follower.update();
            OdometryWatchdog.State state = watchdog.check(follower);
            if (state != OdometryWatchdog.State.OK) {
                clean = false;
                if (state == OdometryWatchdog.State.LOST) {
                    aborted = true;
                    return false;
                }
            }
            Pose pose = follower.pose();
            double ex = target.x() - pose.x();
            double ey = target.y() - pose.y();
            double along = ex * ux + ey * uy;
            double cross = -ex * uy + ey * ux;
            double headingErr = Math.toDegrees(normalize(target.heading() - pose.heading()));

            double fl = motors[0].getPower(), fr = motors[1].getPower();
            double bl = motors[2].getPower(), br = motors[3].getPower();
            double cmdForward = (fl + fr + bl + br) / 4.0;
            double cmdStrafeLeft = (-fl + fr + bl - br) / 4.0;
            double cmdTurn = (-fl + fr - bl + br) / 4.0;
            // The axis command toward the move: + pushes toward the target
            double moveSign = Math.signum(currentDistance);
            double axisCommand = moveSign * (combo.strafe ? cmdStrafeLeft : cmdForward);

            if (trace != null) {
                trace.add(new double[]{t.seconds(), along, cross, headingErr, axisCommand, pose.x(), pose.y()});
            }
            samples.row(clock.seconds(), combo.strafe ? "strafe" : "forward", comboIndex, moveIndex, currentDistance,
                    target.x(), target.y(), pose.x(), pose.y(), Math.toDegrees(pose.heading()), along, cross,
                    headingErr, state, cmdForward, cmdStrafeLeft, cmdTurn);

            telemetry.addData("Combination", "%d / %d: %s", comboIndex + 1, comboCount, combo.label());
            telemetry.addData("Move", "%d: %+.0f in, error %+.2f in (cross %+.2f)", moveIndex + 1, currentDistance,
                    along, cross);
            if (!combo.moves.isEmpty()) {
                MoveResult last = combo.moves.get(combo.moves.size() - 1);
                telemetry.addData("Last move", last.valid
                        ? String.format(Locale.US, "settle %.2f s, final %.2f\", overshoot %.2f\", chatter %.1f/s, cost %.2f",
                        last.settle, last.finalErr, last.overshoot, last.chatter, last.cost) : "not scored (odometry)");
            }
            telemetry.addData("Pinpoint", "%s, %s", watchdog.state(), watchdog.status());
            telemetry.addLine("B: abort");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            packet.put("alongErrorIn", along);
            packet.put("crossErrorIn", cross);
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobotOutline(packet.fieldOverlay(), target, "lime", "target");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            dashboard.sendTelemetryPacket(packet);
        }
        return clean && opModeIsActive();
    }

    /** Fills in the metrics of a move from its trace. */
    private static void score(MoveResult r, List<double[]> trace, double distance) {
        double window = trace.get(trace.size() - 1)[0];

        for (double[] s : trace) {
            if (s[1] < 0.1 * distance) {
                r.t90 = s[0];
                break;
            }
        }
        double minAlong = Double.POSITIVE_INFINITY;
        r.maxCross = 0;
        r.maxHeadingErr = 0;
        for (double[] s : trace) {
            minAlong = Math.min(minAlong, s[1]);
            r.maxCross = Math.max(r.maxCross, Math.abs(s[2]));
            r.maxHeadingErr = Math.max(r.maxHeadingErr, Math.abs(s[3]));
        }
        r.overshoot = Math.max(0.0, -minAlong);

        // Final error (distance from the target) over the last 0.2 s
        double sum = 0, sumAlong = 0;
        int n = 0;
        for (double[] s : trace) {
            if (s[0] >= window - 0.2) {
                sum += Math.hypot(s[1], s[2]);
                sumAlong += s[1];
                n++;
            }
        }
        double[] last = trace.get(trace.size() - 1);
        r.finalErr = n > 0 ? sum / n : Math.hypot(last[1], last[2]);
        r.finalAlong = n > 0 ? sumAlong / n : last[1];

        // Settle: after the last time the position was outside the tolerance
        double lastOutside = 0;
        for (double[] s : trace) {
            if (Math.hypot(s[1], s[2]) > SETTLE_TOL_IN) {
                lastOutside = s[0];
            }
        }
        r.settled = lastOutside < window - 0.2;
        r.settle = r.settled ? lastOutside : window + UNSETTLED_PENALTY_S;

        // Turning points of the along error (it starts by decreasing toward the target)
        List<Double> points = new ArrayList<>();
        double ext = trace.get(0)[1];
        int dir = -1;
        for (double[] s : trace) {
            double e = s[1];
            if (dir < 0) {
                if (e < ext) {
                    ext = e;
                } else if (e > ext + SWING_HYSTERESIS_IN) {
                    points.add(ext);
                    dir = 1;
                    ext = e;
                }
            } else {
                if (e > ext) {
                    ext = e;
                } else if (e < ext - SWING_HYSTERESIS_IN) {
                    points.add(ext);
                    dir = -1;
                    ext = e;
                }
            }
        }
        r.swings = points.size();
        r.firstStop = points.isEmpty() ? last[1] : points.get(0);
        points.add(last[1]);
        r.maxSwing = 0;
        for (int i = 0; i + 1 < points.size(); i++) {
            r.maxSwing = Math.max(r.maxSwing, Math.abs(points.get(i + 1) - points.get(i)));
        }

        // Chatter: sign flips of the axis command while resting (last REST_S), and its mean value
        int flips = 0;
        int lastSign = 0;
        double cmdSum = 0;
        int cmdN = 0;
        for (double[] s : trace) {
            if (s[0] < window - REST_S) {
                continue;
            }
            cmdSum += s[4];
            cmdN++;
            int sign = s[4] > CHATTER_MIN_POWER ? 1 : (s[4] < -CHATTER_MIN_POWER ? -1 : 0);
            if (sign != 0 && lastSign != 0 && sign != lastSign) {
                flips++;
            }
            if (sign != 0) {
                lastSign = sign;
            }
        }
        r.chatter = flips / Math.max(REST_S, 1e-3);
        r.restCommand = cmdN > 0 ? cmdSum / cmdN : 0;

        // Peak speed
        r.peakSpeed = 0;
        for (int i = 1; i < trace.size(); i++) {
            double dt = trace.get(i)[0] - trace.get(i - 1)[0];
            if (dt > 0.003) {
                double ds = Math.hypot(trace.get(i)[5] - trace.get(i - 1)[5], trace.get(i)[6] - trace.get(i - 1)[6]);
                r.peakSpeed = Math.max(r.peakSpeed, ds / dt);
            }
        }

        r.cost = r.settle + W_FINAL * r.finalErr + W_OVERSHOOT * r.overshoot + W_SWING * r.maxSwing
                + W_CHATTER * r.chatter;
    }

    private double voltage() {
        double v = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double x = sensor.getVoltage();
            if (x > 0) {
                v = x;
            }
        }
        return v;
    }

    /** "1, 2.5,3" -> {1, 2.5, 3}; empty array if anything is not a number. */
    private static double[] parse(String list) {
        try {
            List<Double> values = new ArrayList<>();
            for (String p : list.split(",")) {
                if (!p.trim().isEmpty()) {
                    values.add(Double.parseDouble(p.trim()));
                }
            }
            double[] out = new double[values.size()];
            for (int i = 0; i < out.length; i++) {
                out[i] = values.get(i);
            }
            return out;
        } catch (NumberFormatException e) {
            return new double[0];
        }
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
