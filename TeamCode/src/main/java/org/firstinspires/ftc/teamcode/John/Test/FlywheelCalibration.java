package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Hardware.Flywheel;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Flywheel calibration (no balls needed; the robot does not drive): finds the hand-off from the
 * full-power boost to the velocity control that never overshoots, so "at speed" means it is safe to
 * shoot. Same idea as Vex (100% power until close, then a learned holding power); here the holding
 * power is the PIDF's F term.
 *
 * 1. Learn F: run open-loop at each of LEARN_POWERS, let it settle (LEARN_SETTLE_MS), average the RPM
 *    over the last LEARN_AVERAGE_MS. A straight line through (power, RPM) gives the power that holds
 *    the target, and F = 32767 x that power / target counts per second, normalized to
 *    Flywheel.NOMINAL_VOLTAGE (x battery voltage / nominal), since Flywheel scales F by the voltage.
 *    To check the compensation, repeat at several supply voltages: the normalized F should agree.
 *    The hand-off sweep is worst (most overshoot) at the highest voltage: run it on a fresh battery.
 * 2. Sweep the hand-off: for each combination of EXIT_VALUES (Flywheel.BOOST_EXIT_RPM) and I_VALUES
 *    (Flywheel.I), REPEATS times: brake to a stop (cold start), boost at full power, hand over to the
 *    velocity control, and watch for OBSERVE_MS: peak RPM, overshoot, time to ready (within
 *    Flywheel.TOLERANCE_RPM and staying there). With USE_LEARNED_F the sweep runs with the learned F.
 *    First run (r0042, I = 3): the earlier the hand-off, the bigger the overshoot (100 RPM at 100
 *    below, 300 at 600 below): the overshoot comes from P / I acting on the error left at the
 *    hand-off, so with the right F, I should be small.
 * Result: of the combinations whose overshoot stayed within OVERSHOOT_OK_RPM in every trial (once
 * "at speed", it never leaves the band), the one that got ready fastest (worst trial). Y applies it
 * (and the learned F) to Flywheel until the app restarts; telemetry shows the values to put in
 * Flywheel.java. Without Y, the hand-off and I go back to what they were (the learned F stays).
 *
 * gamepad1: X full calibration (learn F, then sweep), B sweep only (current F), dpad up / down target
 * RPM, Y apply the result, A abort.
 * Uses bulk reads (one read of the hub per loop) so the loop is fast enough to see a short overshoot.
 * Logged to /sdcard/FIRST/john_logs/r<run>_john_flywheel_cal_<time>.csv.
 */
@Config("JohnFlywheelCalibration")
@TeleOp(name = "[John] Flywheel Calibration", group = "JohnTest")
public class FlywheelCalibration extends LinearOpMode {

    public static double TARGET_RPM = 2300;
    public static double TARGET_STEP_RPM = 50;
    // ---- F learning ----
    public static String LEARN_POWERS = "0.3,0.4,0.5";
    public static double LEARN_SETTLE_MS = 2500;
    public static double LEARN_AVERAGE_MS = 700;
    public static boolean USE_LEARNED_F = true;
    // ---- Hand-off sweep ----
    public static String EXIT_VALUES = "50,100,150,200";
    public static String I_VALUES = "0,0.5,1,3";
    public static int REPEATS = 2;
    public static double OBSERVE_MS = 3000;
    /**
     * A trial is fine when the peak stays within this of the target: Flywheel.TOLERANCE_RPM, so once
     * "at speed" it never leaves the band (the encoder reads in steps of about 43 RPM).
     */
    public static double OVERSHOOT_OK_RPM = 100;
    /** "Ready" = within Flywheel.TOLERANCE_RPM for this long. */
    public static double READY_HOLD_MS = 100;
    /** Cold start: braked below this RPM, then this much longer. */
    public static double COLD_RPM = 50;
    public static double COLD_EXTRA_MS = 300;
    public static double COLD_TIMEOUT_MS = 8000;

    private enum Phase { IDLE, LEARN, COOL, BOOST_AND_HOLD, DONE }

    private static class Trial {
        double exitRpm;
        double i;
        double peakRpm;
        double handoffRpm = Double.NaN;     // RPM when the boost ended
        double handoffMs = Double.NaN;      // start -> boost ended
        double readyMs = Double.NaN;        // start -> within tolerance for READY_HOLD_MS
        double settledMs = Double.NaN;      // start -> within tolerance until the end of the observation
        double overshoot() { return Math.max(0.0, peakRpm - TARGET_RPM); }
        boolean ok() { return overshoot() <= OVERSHOOT_OK_RPM && !Double.isNaN(settledMs); }
    }

    private static final String LOG_HEADER = "t_s,phase,step,target_rpm,rpm,flywheel_mode,power,f,f_effective,voltage,exit_rpm,event";

    private Flywheel flywheel;
    private CsvLog log;
    private final ElapsedTime clock = new ElapsedTime();
    private final List<Trial> trials = new ArrayList<>();
    private double learnedF = Double.NaN;
    private String learnSummary = "";
    private String result = "";
    private double bestExit = Double.NaN;
    private double bestI = Double.NaN;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        flywheel = new Flywheel(hardwareMap);
        log = new CsvLog("john_flywheel_cal", LOG_HEADER);

        telemetry.addLine("John Flywheel Calibration: no balls, the robot does not drive. Keep hands clear.");
        telemetry.addLine("X learn F + sweep the hand-off, B sweep only, dpad up/down target, Y apply, A abort.");
        telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
        telemetry.update();
        waitForStart();
        gamepad1.resetEdgeDetection();

        Phase phase = Phase.IDLE;
        double[] powers = parse(LEARN_POWERS);
        double[] exits = parse(EXIT_VALUES);
        double[] iValues = parse(I_VALUES);
        int combos = exits.length * iValues.length;
        int learnIndex = 0;
        List<double[]> learned = new ArrayList<>();   // {power, rpm}
        List<Double> window = new ArrayList<>();
        int trialIndex = 0;
        Trial trial = null;
        double savedExit = Flywheel.BOOST_EXIT_RPM;
        double savedEnter = Flywheel.BOOST_ENTER_RPM;
        double savedF = Flywheel.F;
        double savedI = Flywheel.I;
        ElapsedTime phaseTimer = new ElapsedTime();
        ElapsedTime coldTimer = new ElapsedTime();
        ElapsedTime inTolerance = new ElapsedTime();
        boolean wasInTolerance = false;
        ElapsedTime telemetryTimer = new ElapsedTime();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            String event = null;
            boolean full = gamepad1.xWasPressed(), sweepOnly = gamepad1.bWasPressed();
            boolean faster = gamepad1.dpadUpWasPressed(), slower = gamepad1.dpadDownWasPressed();
            boolean apply = gamepad1.yWasPressed(), abort = gamepad1.aWasPressed();
            boolean running = phase != Phase.IDLE && phase != Phase.DONE;

            if (!running && faster) TARGET_RPM += TARGET_STEP_RPM;
            if (!running && slower) TARGET_RPM = Math.max(0, TARGET_RPM - TARGET_STEP_RPM);
            if (abort && running) {
                phase = Phase.IDLE;
                flywheel.setManualPower(null);
                flywheel.setOn(false);
                Flywheel.BOOST_EXIT_RPM = savedExit;
                Flywheel.BOOST_ENTER_RPM = savedEnter;
                Flywheel.F = savedF;
                Flywheel.I = savedI;
                event = "aborted";
            } else if ((full || sweepOnly) && !running) {
                if (!flywheel.closedLoop()) {
                    event = "needs the flywheel encoder (Flywheel.USE_ENCODER)";
                } else {
                    savedExit = Flywheel.BOOST_EXIT_RPM;
                    savedEnter = Flywheel.BOOST_ENTER_RPM;
                    savedF = Flywheel.F;
                    savedI = Flywheel.I;
                    trials.clear();
                    result = "";
                    bestExit = Double.NaN;
                    bestI = Double.NaN;
                    trialIndex = 0;
                    flywheel.setTargetRpm(TARGET_RPM);
                    if (full) {
                        learnIndex = 0;
                        learned.clear();
                        window.clear();
                        learnedF = Double.NaN;
                        learnSummary = "";
                        flywheel.setManualPower(powers[0]);
                        phase = Phase.LEARN;
                        event = "learn F at " + powers[0];
                    } else {
                        phase = Phase.COOL;
                        flywheel.setBrakeWhenOff(true);
                        flywheel.setOn(false);
                        event = "sweep (current F " + Flywheel.F + ")";
                    }
                    phaseTimer.reset();
                    coldTimer.reset();
                }
            }
            if (apply && !Double.isNaN(bestExit) && !running) {
                applyResult();
                savedExit = Flywheel.BOOST_EXIT_RPM;
                savedEnter = Flywheel.BOOST_ENTER_RPM;
                savedI = Flywheel.I;
                event = "applied: " + result;
            }

            flywheel.update();
            double rpm = flywheel.rpm();
            boolean withinTol = Math.abs(rpm - TARGET_RPM) <= Flywheel.TOLERANCE_RPM;
            if (withinTol && !wasInTolerance) {
                inTolerance.reset();
            }
            wasInTolerance = withinTol;

            switch (phase) {
                case LEARN:
                    if (phaseTimer.milliseconds() >= LEARN_SETTLE_MS - LEARN_AVERAGE_MS) {
                        window.add(rpm);
                    }
                    if (phaseTimer.milliseconds() >= LEARN_SETTLE_MS) {
                        double avg = average(window);
                        learned.add(new double[]{powers[learnIndex], avg});
                        event = String.format(Locale.US, "power %.2f -> %.0f RPM", powers[learnIndex], avg);
                        window.clear();
                        learnIndex++;
                        phaseTimer.reset();
                        if (learnIndex < powers.length) {
                            flywheel.setManualPower(powers[learnIndex]);
                        } else {
                            flywheel.setManualPower(null);
                            fitF(learned);
                            if (USE_LEARNED_F && !Double.isNaN(learnedF)) {
                                Flywheel.F = learnedF;
                            }
                            phase = Phase.COOL;
                            flywheel.setBrakeWhenOff(true);
                            flywheel.setOn(false);
                            coldTimer.reset();
                        }
                    }
                    break;
                case COOL:
                    if (rpm > COLD_RPM) {
                        coldTimer.reset();
                    }
                    if (coldTimer.milliseconds() >= COLD_EXTRA_MS || phaseTimer.milliseconds() > COLD_TIMEOUT_MS) {
                        if (trialIndex >= combos * Math.max(1, REPEATS)) {
                            finish();
                            // Back to the previous hand-off and I until Y applies the result
                            Flywheel.BOOST_EXIT_RPM = savedExit;
                            Flywheel.BOOST_ENTER_RPM = savedEnter;
                            Flywheel.I = savedI;
                            phase = Phase.DONE;
                            flywheel.setBrakeWhenOff(false);
                            event = result;
                            break;
                        }
                        int combo = trialIndex % combos;
                        trial = new Trial();
                        trial.exitRpm = exits[combo % exits.length];
                        trial.i = iValues[combo / exits.length];
                        trial.peakRpm = 0;
                        trials.add(trial);
                        Flywheel.BOOST_EXIT_RPM = trial.exitRpm;
                        Flywheel.BOOST_ENTER_RPM = Math.max(trial.exitRpm + 150, savedEnter);
                        Flywheel.I = trial.i;
                        flywheel.setOn(true);
                        phase = Phase.BOOST_AND_HOLD;
                        phaseTimer.reset();
                        event = String.format(Locale.US, "trial %d: hand-off %.0f RPM below, I %.2f",
                                trialIndex + 1, trial.exitRpm, trial.i);
                    }
                    break;
                case BOOST_AND_HOLD:
                    double t = phaseTimer.milliseconds();
                    trial.peakRpm = Math.max(trial.peakRpm, rpm);
                    if (Double.isNaN(trial.handoffMs) && !flywheel.boosting() && t > 50) {
                        trial.handoffMs = t;
                        trial.handoffRpm = rpm;
                    }
                    if (Double.isNaN(trial.readyMs) && withinTol && inTolerance.milliseconds() >= READY_HOLD_MS) {
                        trial.readyMs = t;
                    }
                    if (!withinTol) {
                        trial.settledMs = Double.NaN;
                    } else if (Double.isNaN(trial.settledMs)) {
                        trial.settledMs = t;
                    }
                    if (!Double.isNaN(trial.handoffMs) && t - trial.handoffMs >= OBSERVE_MS) {
                        event = describe(trial);
                        flywheel.setOn(false);
                        trialIndex++;
                        phase = Phase.COOL;
                        phaseTimer.reset();
                        coldTimer.reset();
                    }
                    break;
                default:
                    break;
            }
            if (flywheel.event() != null && event == null) {
                event = flywheel.event();
            }

            if (telemetryTimer.milliseconds() > 100) {
                telemetryTimer.reset();
                showTelemetry(phase, rpm, trialIndex, combos * Math.max(1, REPEATS));
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("targetRpm", TARGET_RPM);
                packet.put("rpm", rpm);
                dashboard.sendTelemetryPacket(packet);
            }
            log.row(clock.seconds(), phase, trialIndex, TARGET_RPM, rpm, flywheel.mode(), flywheel.power(), Flywheel.F,
                    flywheel.effectiveF(), flywheel.voltage(), Flywheel.BOOST_EXIT_RPM, event);
        }
        flywheel.setManualPower(null);
        flywheel.stop();
        log.close();
    }

    /** Least-squares line RPM = a x power + b, then the power that holds the target and its F. */
    private void fitF(List<double[]> points) {
        int n = points.size();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] p : points) {
            sx += p[0];
            sy += p[1];
            sxx += p[0] * p[0];
            sxy += p[0] * p[1];
        }
        double den = n * sxx - sx * sx;
        if (n < 2 || Math.abs(den) < 1e-9) {
            learnSummary = "not enough points to learn F";
            return;
        }
        double a = (n * sxy - sx * sy) / den;
        double b = (sy - a * sx) / n;
        double holdPower = (TARGET_RPM - b) / a;
        double targetTicksPerS = TARGET_RPM / 60.0 * Flywheel.TICKS_PER_REV * Flywheel.GEARING;
        double fHere = 32767.0 * holdPower / targetTicksPerS;
        // Normalized to the nominal voltage (Flywheel scales it back by nominal / actual voltage)
        double v = flywheel.voltage();
        learnedF = Flywheel.VOLTAGE_COMPENSATION ? fHere * v / Flywheel.NOMINAL_VOLTAGE : fHere;
        learnSummary = String.format(Locale.US, "RPM = %.0f x power %+.0f; holding %.0f RPM needs power %.3f at %.2f V "
                + "-> F %.2f here, %.2f at %.0f V (was %.2f)", a, b, TARGET_RPM, holdPower, v, fHere, learnedF,
                Flywheel.NOMINAL_VOLTAGE, Flywheel.F);
        log.row(clock.seconds(), "learned", null, TARGET_RPM, null, null, holdPower, learnedF, fHere, v, null, learnSummary);
    }

    /**
     * Of the (hand-off, I) combinations with every trial within OVERSHOOT_OK_RPM and settled, the one
     * whose slowest trial got ready first.
     */
    private void finish() {
        double bestReady = Double.POSITIVE_INFINITY;
        bestExit = Double.NaN;
        bestI = Double.NaN;
        for (double exit : parse(EXIT_VALUES)) {
            for (double i : parse(I_VALUES)) {
                boolean allOk = true;
                boolean any = false;
                double readyWorst = 0;
                for (Trial t : trials) {
                    if (t.exitRpm == exit && t.i == i) {
                        any = true;
                        allOk &= t.ok() && !Double.isNaN(t.readyMs);
                        if (!Double.isNaN(t.readyMs)) {
                            readyWorst = Math.max(readyWorst, t.readyMs);
                        }
                    }
                }
                if (any && allOk && readyWorst < bestReady) {
                    bestReady = readyWorst;
                    bestExit = exit;
                    bestI = i;
                }
            }
        }
        if (Double.isNaN(bestExit)) {
            result = "no combination stayed within " + OVERSHOOT_OK_RPM + " RPM: try smaller I_VALUES (or lower P)";
        } else {
            result = String.format(Locale.US, "best: BOOST_EXIT_RPM %.0f, I %.2f (ready in <= %.0f ms), F %.2f. "
                    + "Y applies; put them in Flywheel.java", bestExit, bestI, bestReady, Flywheel.F);
        }
        log.row(clock.seconds(), "result", null, TARGET_RPM, null, null, null, Flywheel.F, flywheel.effectiveF(),
                flywheel.voltage(), bestExit, result);
    }

    private void applyResult() {
        Flywheel.BOOST_EXIT_RPM = bestExit;
        Flywheel.I = bestI;
        Flywheel.BOOST_ENTER_RPM = Math.max(bestExit + 150, Flywheel.BOOST_ENTER_RPM);
        if (USE_LEARNED_F && !Double.isNaN(learnedF)) {
            Flywheel.F = learnedF;
        }
    }

    private String describe(Trial t) {
        return String.format(Locale.US, "hand-off %3.0f below, I %.2f: boost ended at %.0f RPM after %.0f ms, "
                        + "peak %.0f (overshoot %.0f), ready %s, settled %s%s",
                t.exitRpm, t.i, t.handoffRpm, t.handoffMs, t.peakRpm, t.overshoot(),
                Double.isNaN(t.readyMs) ? "never" : String.format(Locale.US, "%.0f ms", t.readyMs),
                Double.isNaN(t.settledMs) ? "no" : String.format(Locale.US, "%.0f ms", t.settledMs),
                t.ok() ? "" : "  X");
    }

    private void showTelemetry(Phase phase, double rpm, int trialIndex, int trialCount) {
        telemetry.addData("Target", "%.0f RPM (dpad up/down when idle), actual %.0f RPM, %s",
                TARGET_RPM, rpm, flywheel.mode());
        telemetry.addData("Phase", phase == Phase.IDLE ? "X: learn F + sweep, B: sweep only"
                : String.format(Locale.US, "%s, trial %d / %d", phase, Math.min(trialIndex + 1, trialCount), trialCount));
        telemetry.addData("F", "%.2f at %.0f V, %.2f now at %.2f V%s", Flywheel.F, Flywheel.NOMINAL_VOLTAGE,
                flywheel.effectiveF(), flywheel.voltage(), learnSummary.isEmpty() ? "" : "  (" + learnSummary + ")");
        for (Trial t : trials) {
            telemetry.addLine("  " + describe(t));
        }
        if (!result.isEmpty()) {
            telemetry.addData("Result", result);
        }
        telemetry.addLine("Y apply, A abort");
        telemetry.update();
    }

    private static double average(List<Double> values) {
        double sum = 0;
        for (double v : values) {
            sum += v;
        }
        return values.isEmpty() ? Double.NaN : sum / values.size();
    }

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
}
