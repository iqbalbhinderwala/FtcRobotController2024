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
import org.firstinspires.ftc.teamcode.John.Pedro.JohnTuning;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Automatic sweep of John's heading controller values over turns of different sizes (and so
 * different turn speeds).
 *
 * For every combination of KP_VALUES x KS_VALUES x BRAKE_A_VALUES x BRAKE_B_VALUES (comma-separated
 * lists, editable on the Dashboard before START), it swaps in the controller and turns in place by
 * each of ANGLES_DEG, first + then - (so it ends where it started), REPEATS times. Each turn is
 * watched for TURN_WINDOW_S and scored:
 *   cost = time to settle within SETTLE_TOL_DEG (+ UNSETTLED_PENALTY_S if it never does)
 *          + W_FINAL x final error + W_OVERSHOOT x overshoot + W_SWING x largest swing after the
 *          first stop (all in degrees).
 * At the end the combinations are ranked by mean cost. X applies the best one to JohnTuning (until
 * the app restarts) and holds with it. B aborts and ranks what has been done so far.
 *
 * The robot only turns in place: give it room to spin. OdometryWatchdog pauses the drive on bad
 * Pinpoint reads; turns that saw a pause are not scored, and a LOST odometry aborts the sweep.
 *
 * Logs (/sdcard/FIRST/john_logs/), sharing one run number: r<run>_john_heading_cal_samples (every
 * loop), _turns (one row per turn), _ranking (per combination and per combination x angle).
 */
@Config
@TeleOp(name = "[John] Heading Calibration", group = "JohnTest")
public class HeadingCalibration extends LinearOpMode {

    // ---- Sweep (comma-separated lists) ----
    public static String KP_VALUES = "5.2";
    public static String KS_VALUES = "0";
    public static String BRAKE_A_VALUES = "0.045,0.05";
    public static String BRAKE_B_VALUES = "0.0046,0.006,0.0075";
    public static double KS_DEADBAND_DEG = 0.25;
    /** Avoid 180: the shorter way round is ambiguous there. */
    public static String ANGLES_DEG = "15,45,90,135";
    public static int REPEATS = 2;

    // ---- Timing ----
    public static double TURN_WINDOW_S = 1.5;
    public static double COMBO_SETTLE_S = 1.0;

    // ---- Scoring ----
    public static double SETTLE_TOL_DEG = 0.5;
    public static double UNSETTLED_PENALTY_S = 0.5;
    public static double W_FINAL = 0.5;
    public static double W_OVERSHOOT = 0.2;
    public static double W_SWING = 0.2;
    /** Smallest heading move counted as a swing (filters sensor noise). */
    public static double SWING_HYSTERESIS_DEG = 0.1;

    /** Start pose, FTC field coordinates (only places the robot on the Dashboard field). */
    public static double START_X_IN = 36.0;
    public static double START_Y_IN = 36.0;
    public static double START_HEADING_DEG = 180.0;

    private static final String SAMPLE_HEADER = "t_s,combo,turn,angle_deg,target_h_deg,h_deg,err_deg,"
            + "watchdog,p_fl,p_fr,p_bl,p_br";
    private static final String TURN_HEADER = "t_s,combo,turn,kp,ks,ks_deadband_deg,brake_a,brake_b,angle_deg,"
            + "valid,t90_s,settle_s,settled,overshoot_deg,first_stop_deg,max_swing_deg,swings,final_deg,"
            + "peak_rate_dps,cost,voltage";
    private static final String RANKING_HEADER = "rank,kp,ks,ks_deadband_deg,brake_a,brake_b,angle_deg,turns,"
            + "cost,t90_s,settle_s,settled_frac,overshoot_deg,max_swing_deg,final_deg";

    private Follower follower;
    private OdometryWatchdog watchdog;
    private DcMotor[] motors;
    private FtcDashboard dashboard;
    private CsvLog samples;
    private final ElapsedTime clock = new ElapsedTime();
    private Pose target;
    private boolean aborted;
    private int comboIndex;
    private int turnIndex;
    private double currentAngle;

    /** One controller combination and the turns scored with it. */
    private static class Combo {
        final double kp, ks, brakeA, brakeB;
        final List<TurnResult> turns = new ArrayList<>();

        Combo(double kp, double ks, double brakeA, double brakeB) {
            this.kp = kp;
            this.ks = ks;
            this.brakeA = brakeA;
            this.brakeB = brakeB;
        }

        String label() {
            return String.format(Locale.US, "KP %.2f KS %.3f A %.3f B %.4f", kp, ks, brakeA, brakeB);
        }
    }

    /** Metrics of one turn. Errors are signed toward the turn: + = short of the target, - = past. */
    private static class TurnResult {
        double angleDeg;
        boolean valid = true;
        double t90 = Double.NaN;
        double settle;
        boolean settled;
        double overshoot;
        double firstStop;
        double maxSwing;
        int swings;
        double finalErr;
        double peakRate;
        double cost;
    }

    /** Averages of a set of turns. */
    private static class Stats {
        int n;
        double cost, t90, settle, settledFrac, overshoot, maxSwing, finalErr;

        static Stats of(List<TurnResult> turns, double angleOrNaN) {
            Stats s = new Stats();
            int n90 = 0;
            for (TurnResult r : turns) {
                if (!r.valid || (!Double.isNaN(angleOrNaN) && r.angleDeg != angleOrNaN)) {
                    continue;
                }
                s.n++;
                s.cost += r.cost;
                s.settle += r.settle;
                s.settledFrac += r.settled ? 1 : 0;
                s.overshoot += r.overshoot;
                s.maxSwing += r.maxSwing;
                s.finalErr += r.finalErr;
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
            }
            s.t90 = n90 > 0 ? s.t90 / n90 : Double.NaN;
            return s;
        }
    }

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        double[] kps = parse(KP_VALUES);
        double[] kss = parse(KS_VALUES);
        double[] as = parse(BRAKE_A_VALUES);
        double[] bs = parse(BRAKE_B_VALUES);
        double[] angles = parse(ANGLES_DEG);
        List<Combo> combos = new ArrayList<>();
        for (double kp : kps) {
            for (double ks : kss) {
                for (double a : as) {
                    for (double b : bs) {
                        combos.add(new Combo(kp, ks, a, b));
                    }
                }
            }
        }
        int turnsPerCombo = angles.length * 2 * Math.max(1, REPEATS);
        double estimateS = combos.size() * (COMBO_SETTLE_S + turnsPerCombo * TURN_WINDOW_S);

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
        samples = new CsvLog("john_heading_cal_samples", SAMPLE_HEADER, run);
        CsvLog turnLog = new CsvLog("john_heading_cal_turns", TURN_HEADER, run);
        CsvLog rankingLog = new CsvLog("john_heading_cal_ranking", RANKING_HEADER, run);

        boolean listsOk = kps.length > 0 && kss.length > 0 && as.length > 0 && bs.length > 0 && angles.length > 0;
        telemetry.addLine(listsOk ? "Heading calibration: the robot will turn in place. Give it room."
                : "ERROR: a value list is empty or not a comma-separated list of numbers.");
        telemetry.addData("Combinations", "%d x %d turns (angles %s, +/-, x%d)",
                combos.size(), turnsPerCombo, ANGLES_DEG, Math.max(1, REPEATS));
        telemetry.addData("Estimated time", "%.0f s", estimateS);
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
            gains.headingKp = combo.kp;
            gains.headingKs = combo.ks;
            gains.headingKsDeadbandDeg = KS_DEADBAND_DEG;
            gains.headingBrakeA = combo.brakeA;
            gains.headingBrakeB = combo.brakeB;
            follower.setAlgorithm(new Foresight(Constants.foresightConfig(gains)));
            follower.hold(target);
            currentAngle = 0;
            run(COMBO_SETTLE_S, null, combo, combos.size());

            turnIndex = 0;
            for (int rep = 0; rep < Math.max(1, REPEATS) && !aborted; rep++) {
                for (double angle : angles) {
                    for (int dir : new int[]{1, -1}) {
                        if (aborted) {
                            break;
                        }
                        currentAngle = dir * angle;
                        target = target.withHeading(target.heading() + Math.toRadians(currentAngle));
                        follower.hold(target);
                        TurnResult r = turn(combo, combos.size(), angle);
                        combo.turns.add(r);
                        turnLog.row(clock.seconds(), comboIndex, turnIndex, combo.kp, combo.ks, KS_DEADBAND_DEG,
                                combo.brakeA, combo.brakeB, currentAngle, r.valid ? 1 : 0, r.t90, r.settle,
                                r.settled ? 1 : 0, r.overshoot, r.firstStop, r.maxSwing, r.swings, r.finalErr,
                                r.peakRate, r.cost, voltage());
                        turnIndex++;
                    }
                }
            }
        }

        // Ranking
        List<Combo> ranked = new ArrayList<>();
        for (Combo c : combos) {
            if (Stats.of(c.turns, Double.NaN).n > 0) {
                ranked.add(c);
            }
        }
        Collections.sort(ranked, (x, y) -> Double.compare(Stats.of(x.turns, Double.NaN).cost,
                Stats.of(y.turns, Double.NaN).cost));
        for (int i = 0; i < ranked.size(); i++) {
            Combo c = ranked.get(i);
            Stats all = Stats.of(c.turns, Double.NaN);
            rankingLog.row(i + 1, c.kp, c.ks, KS_DEADBAND_DEG, c.brakeA, c.brakeB, "all", all.n, all.cost, all.t90,
                    all.settle, all.settledFrac, all.overshoot, all.maxSwing, all.finalErr);
            for (double angle : angles) {
                Stats s = Stats.of(c.turns, angle);
                rankingLog.row(i + 1, c.kp, c.ks, KS_DEADBAND_DEG, c.brakeA, c.brakeB, angle, s.n, s.cost, s.t90,
                        s.settle, s.settledFrac, s.overshoot, s.maxSwing, s.finalErr);
            }
        }
        samples.close();
        turnLog.close();
        rankingLog.close();

        // Results: hold, show the top of the ranking, X applies the best
        boolean applied = false;
        boolean lastX = false;
        while (opModeIsActive()) {
            boolean x = gamepad1.x;
            if (x && !lastX && !ranked.isEmpty() && !watchdog.lost()) {
                Combo best = ranked.get(0);
                JohnTuning.HEADING_KP = best.kp;
                JohnTuning.HEADING_KS = best.ks;
                JohnTuning.HEADING_KS_DEADBAND_DEG = KS_DEADBAND_DEG;
                JohnTuning.HEADING_BRAKE_A = best.brakeA;
                JohnTuning.HEADING_BRAKE_B = best.brakeB;
                follower.setAlgorithm(new Foresight(Constants.foresightConfig()));
                follower.hold(target);
                applied = true;
            }
            lastX = x;
            follower.update();
            watchdog.check(follower);

            telemetry.addLine(aborted ? "Sweep ABORTED: ranking of the turns done so far" : "Sweep done");
            if (watchdog.lost()) {
                telemetry.addLine("ODOMETRY LOST: " + watchdog.reason());
            }
            telemetry.addLine("rank  cost  | settle  final  overshoot  swing (deg, s)  | combination");
            for (int i = 0; i < Math.min(5, ranked.size()); i++) {
                Combo c = ranked.get(i);
                Stats s = Stats.of(c.turns, Double.NaN);
                telemetry.addLine(String.format(Locale.US, "%d  %.2f  | %.2f s  %.2f  %.2f  %.2f  | %s  (%d turns)",
                        i + 1, s.cost, s.settle, s.finalErr, s.overshoot, s.maxSwing, c.label(), s.n));
            }
            if (!ranked.isEmpty()) {
                Combo best = ranked.get(0);
                for (double angle : angles) {
                    Stats s = Stats.of(best.turns, angle);
                    telemetry.addLine(String.format(Locale.US, "  best at %.0f deg: settle %.2f s, final %.2f, "
                            + "overshoot %.2f, swing %.2f, 90%% at %.2f s", angle, s.settle, s.finalErr,
                            s.overshoot, s.maxSwing, s.t90));
                }
            }
            telemetry.addLine(applied ? "Best values applied to JohnTuning (until the app restarts)."
                    : "X: apply the best values to JohnTuning (until the app restarts).");
            telemetry.addData("Logs", "john_logs/%s (+ _turns, _ranking)", samples.fileName());
            telemetry.update();
        }
    }

    /** Runs one turn for TURN_WINDOW_S and scores it. */
    private TurnResult turn(Combo combo, int comboCount, double angleDeg) {
        List<double[]> trace = new ArrayList<>();   // {t, signed error toward the turn (+ short), heading deg}
        TurnResult r = new TurnResult();
        r.angleDeg = angleDeg;
        boolean ok = run(TURN_WINDOW_S, trace, combo, comboCount);
        if (!ok || trace.size() < 10) {
            r.valid = false;
            return r;
        }
        score(r, trace, angleDeg);
        return r;
    }

    /**
     * Runs the loop for a while. With a trace, records {t, error signed toward the turn, heading}.
     * @return false if the watchdog paused or lost odometry during it (the turn is not scored)
     */
    private boolean run(double seconds, List<double[]> trace, Combo combo, int comboCount) {
        ElapsedTime t = new ElapsedTime();
        boolean clean = true;
        double sign = Math.signum(currentAngle);
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
            double err = Math.toDegrees(normalize(target.heading() - pose.heading()));
            if (trace != null) {
                trace.add(new double[]{t.seconds(), sign * err, Math.toDegrees(pose.heading())});
            }
            samples.row(clock.seconds(), comboIndex, turnIndex, currentAngle, Math.toDegrees(target.heading()),
                    Math.toDegrees(pose.heading()), err, state, motors[0].getPower(), motors[1].getPower(),
                    motors[2].getPower(), motors[3].getPower());

            telemetry.addData("Combination", "%d / %d: %s", comboIndex + 1, comboCount, combo.label());
            telemetry.addData("Turn", "%d: %+.0f deg, error %+.2f deg", turnIndex + 1, currentAngle, err);
            if (!combo.turns.isEmpty()) {
                TurnResult last = combo.turns.get(combo.turns.size() - 1);
                telemetry.addData("Last turn", last.valid
                        ? String.format(Locale.US, "settle %.2f s, final %.2f, overshoot %.2f, swing %.2f deg, cost %.2f",
                        last.settle, last.finalErr, last.overshoot, last.maxSwing, last.cost) : "not scored (odometry)");
            }
            telemetry.addData("Pinpoint", "%s, %s", watchdog.state(), watchdog.status());
            telemetry.addLine("B: abort");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            packet.put("headingErrorDeg", err);
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobotOutline(packet.fieldOverlay(), target, "lime", "target");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            dashboard.sendTelemetryPacket(packet);
        }
        return clean && opModeIsActive();
    }

    /** Fills in the metrics of a turn from its trace (error signed toward the turn: + short, - past). */
    private static void score(TurnResult r, List<double[]> trace, double angleDeg) {
        double window = trace.get(trace.size() - 1)[0];

        // Time to 90 % of the turn
        for (double[] s : trace) {
            if (s[1] < 0.1 * angleDeg) {
                r.t90 = s[0];
                break;
            }
        }
        // Overshoot past the target
        double minErr = Double.POSITIVE_INFINITY;
        for (double[] s : trace) {
            minErr = Math.min(minErr, s[1]);
        }
        r.overshoot = Math.max(0.0, -minErr);

        // Final error: mean |error| over the last 0.2 s
        double sum = 0;
        int n = 0;
        for (double[] s : trace) {
            if (s[0] >= window - 0.2) {
                sum += Math.abs(s[1]);
                n++;
            }
        }
        r.finalErr = n > 0 ? sum / n : Math.abs(trace.get(trace.size() - 1)[1]);

        // Settle: after the last time the error was outside the tolerance
        double lastOutside = 0;
        for (double[] s : trace) {
            if (Math.abs(s[1]) > SETTLE_TOL_DEG) {
                lastOutside = s[0];
            }
        }
        r.settled = lastOutside < window - 0.2;
        r.settle = r.settled ? lastOutside : window + UNSETTLED_PENALTY_S;

        // Turning points of the error (it starts by decreasing toward the target)
        List<Double> points = new ArrayList<>();
        double ext = trace.get(0)[1];
        int dir = -1;
        for (double[] s : trace) {
            double e = s[1];
            if (dir < 0) {
                if (e < ext) {
                    ext = e;
                } else if (e > ext + SWING_HYSTERESIS_DEG) {
                    points.add(ext);
                    dir = 1;
                    ext = e;
                }
            } else {
                if (e > ext) {
                    ext = e;
                } else if (e < ext - SWING_HYSTERESIS_DEG) {
                    points.add(ext);
                    dir = -1;
                    ext = e;
                }
            }
        }
        double last = trace.get(trace.size() - 1)[1];
        r.swings = points.size();
        r.firstStop = points.isEmpty() ? last : points.get(0);
        points.add(last);
        r.maxSwing = 0;
        for (int i = 0; i + 1 < points.size(); i++) {
            r.maxSwing = Math.max(r.maxSwing, Math.abs(points.get(i + 1) - points.get(i)));
        }

        // Peak turn rate
        r.peakRate = 0;
        for (int i = 1; i < trace.size(); i++) {
            double dt = trace.get(i)[0] - trace.get(i - 1)[0];
            if (dt > 0.003) {
                double dh = Math.abs(normalizeDeg(trace.get(i)[2] - trace.get(i - 1)[2]));
                r.peakRate = Math.max(r.peakRate, dh / dt);
            }
        }

        r.cost = r.settle + W_FINAL * r.finalErr + W_OVERSHOOT * r.overshoot + W_SWING * r.maxSwing;
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
            String[] parts = list.split(",");
            List<Double> values = new ArrayList<>();
            for (String p : parts) {
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

    private static double normalizeDeg(double deg) {
        return Math.toDegrees(normalize(Math.toRadians(deg)));
    }
}
