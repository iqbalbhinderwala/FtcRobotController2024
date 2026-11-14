package org.firstinspires.ftc.teamcode.John.Test;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Aimer;
import org.firstinspires.ftc.teamcode.John.Auto.AutoSettings;
import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Hardware.Flywheel;
import org.firstinspires.ftc.teamcode.John.Hardware.Intake;
import org.firstinspires.ftc.teamcode.John.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.ShootingZone;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;
import org.firstinspires.ftc.teamcode.John.Vision.CellTracker;
import org.firstinspires.ftc.teamcode.John.Vision.HiveCamera;
import org.firstinspires.ftc.teamcode.John.Vision.HiveTags;
import org.firstinspires.ftc.teamcode.John.Vision.TagLocalizer;
import org.firstinspires.ftc.teamcode.John.Vision.VisionAim;
import org.firstinspires.ftc.teamcode.John.Vision.WallFinder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Shot calibration: finds the flywheel speed for each distance and angle to the up CELL, for
 * Shooter.RPM_DISTANCES_IN / RPM_VALUES. Every volley is one row of the results log; the coverage
 * (ShotCoverage) adds up all results logs on the robot, so each run carries on from the last.
 *
 * START: as in the TeleOp, from the tags in INIT: flush on a back wall (WallFinder) is exact, else
 * the vision pose (TagLocalizer.solvePose), else the Game Setup start wall. The distance does not
 * need an exact position (the CELL is placed by the camera from the same odometry pose, so an
 * offset cancels out); the heading matters (the angle off the CELL axis). Back held 1 s: re-position
 * from the tags. The alliance (our HIVE) comes from Game Setup, else B / X in INIT.
 *
 * The loop, on gamepad1:
 *  1. Y held: drive to the selected test spot (the most useful one not covered yet, ShotCoverage;
 *     right stick button selects the next one). Letting go stops. There the speed is set to the suggested one.
 *  2. Load BALLS balls (bumpers: intakes on / off; B held: reverse).
 *  3. A held: aims at the up CELL from where John stands (any spot can be tested, the match zone
 *     check is not applied), then fires BALLS + SPARE_STEPS indexer steps at the set speed. Letting go
 *     cancels. Dpad: speed +/- RPM_STEP (up / down), +/- RPM_BIG_STEP (right / left); X: the suggested
 *     speed. If the HIVE tips during the volley, the volley stops (the rest would go at a moving CELL).
 *  4. RESULT: dpad up / down how many went in (starts at BALLS), X the miss type, A saves, Back
 *     discards. The next suggested speed is set.
 *
 * Logged to /sdcard/FIRST/john_v2_logs/r<run>_john_shot_results_<time>.csv (one row per volley) and
 * r<run>_john_shot_cal_<time>.csv (every loop).
 */
@Config("JohnShotCalibration")
@TeleOp(name = "[John] Shot Calibration", group = "JohnTest")
public class ShotCalibration extends LinearOpMode {

    public static int BALLS = 4;
    /** Extra indexer steps in case a ball comes late. */
    public static int SPARE_STEPS = 1;
    public static double RPM_STEP = 25;
    public static double RPM_BIG_STEP = 100;
    public static boolean STOP_ON_TIP = true;
    public static double DRIVE_SCALE = 0.6;
    public static double TURN_SCALE = 0.5;
    public static double STICK_DEADBAND = 0.05;
    public static double CANCEL_STICK = 0.3;
    public static double DRIVE_TIMEOUT_MS = 8000;
    /** Y does nothing this close to the spot (a tiny path only makes the robot shake). */
    public static double ARRIVED_IN = 1.5;
    public static double ARRIVED_DEG = 3.0;
    public static int START_SAMPLES = 60;
    public static int MIN_SAMPLES = 5;
    public static double RELOCALIZE_MS = 1000;
    /** Re-position: keep the Pinpoint heading when the vision heading is within this of it. */
    public static double KEEP_HEADING_DEG = 10.0;
    public static int POSE_MIN_TAGS = 3;
    public static double POSE_MIN_SPAN_IN = 8.0;
    /** Warn when the camera has not seen our HIVE for this long. */
    public static double NO_TAGS_WARN_MS = 3000;

    private static final String[] MISSES = {"none", "short", "long", "left", "right", "bounced out", "mixed"};

    private enum State { IDLE, DRIVE, AIM, FIRE, RESULT }

    private static final String LOG_HEADER = "t_s,state,x,y,h_deg,cell,distance,off_axis_deg,spot,"
            + "rpm_set,fw_rpm,fw_mode,fw_amps,battery_v,idx_steps,idx_deg,idx_target_deg,idx_state,tags,hive,up_height_in,event";
    private static final String RESULT_HEADER = "volley,t_s,alliance,cell,x,y,h_deg,distance_in,cam_distance_in,"
            + "off_axis_deg,nom_distance_in,nom_off_axis_deg,spot,rpm_set,table_rpm,fire_tol_rpm,rpm_at_balls,min_rpm,shot_interval_ms,step_ms,gap_ms,"
            + "steps_done,tipped,made,balls,"
            + "miss,battery_v,fw_f,aim";

    private boolean red = true;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Follower follower = Constants.create(hardwareMap);
        OdometryWatchdog watchdog = new OdometryWatchdog(hardwareMap, "shot_cal");
        HiveCamera camera = new HiveCamera(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Flywheel flywheel = shooter.flywheel();
        Intake intake = new Intake(hardwareMap);
        ShotCoverage coverage = new ShotCoverage();
        coverage.loadLogs();

        AutoSettings settings = new AutoSettings(blackboard);
        boolean haveSetup = AutoSettings.isSaved(blackboard);
        if (!haveSetup) {
            AutoSettings fromFile = AutoSettings.loadFile();
            if (fromFile != null) {
                settings = fromFile;
                haveSetup = true;
            }
        }
        red = settings.alliance() == AutoSettings.Alliance.RED;
        int run = CsvLog.nextRunNumber();
        CsvLog log = new CsvLog("john_shot_cal", LOG_HEADER, run);
        CsvLog results = new CsvLog("john_shot_results", RESULT_HEADER, run);

        // ---- INIT: the start pose from the tags ----
        WallFinder walls = new WallFinder(START_SAMPLES);
        List<Pose> initVision = new ArrayList<>();
        gamepad1.resetEdgeDetection();
        while (opModeInInit()) {
            if (!haveSetup && gamepad1.bWasPressed()) red = true;
            if (!haveSetup && gamepad1.xWasPressed()) red = false;
            HiveCamera.Frame frame = camera.poll();
            walls.add(frame);
            Pose seen = TagLocalizer.solvePose(frame, POSE_MIN_TAGS, POSE_MIN_SPAN_IN);
            if (seen != null) {
                initVision.add(seen);
                if (initVision.size() > START_SAMPLES) initVision.remove(0);
            }
            boolean fix = walls.found(MIN_SAMPLES) != null || initVision.size() >= MIN_SAMPLES;
            telemetry.addLine(fix ? "READY: press START" : "!!! NO TAG FIX: put the camera wall flush on a back wall, facing the HIVE");
            telemetry.addData("Alliance", "%s HIVE%s", red ? "RED" : "BLUE", haveSetup ? "" : "   (B red, X blue)");
            telemetry.addData("Start", "%s  (%s)", Drawing.format(startPose(walls, initVision, settings)),
                    startSource(walls, initVision));
            telemetry.addData("Coverage", "%d of %d test spots still to do; %s", coverage.uncoveredCount(),
                    coverage.spotCount(), coverage.loaded());
            if (!camera.available()) telemetry.addLine("!!! NO CAMERA: " + camera.status());
            telemetry.update();
        }
        if (isStopRequested()) {
            camera.stop();
            log.close();
            results.close();
            watchdog.close();
            return;
        }

        Pose start = startPose(walls, initVision, settings);
        follower.setPose(start);
        watchdog.poseSet();
        follower.update();
        CellTracker tracker = new CellTracker(red);
        Aimer aimer = new Aimer(follower, tracker, camera.available());
        gamepad1.resetEdgeDetection();
        ElapsedTime clock = new ElapsedTime();
        ElapsedTime driveTimer = new ElapsedTime();
        log.row(0.0, State.IDLE, start.x(), start.y(), Math.toDegrees(start.heading()), null, null, null, null,
                null, null, null, null, flywheel.voltage(), null, null, null, null,
                "start " + (red ? "red" : "blue") + " at " + Drawing.format(start) + " (" + startSource(walls, initVision)
                        + "); " + coverage.loaded());

        State state = State.IDLE;
        boolean aLatched = false;
        boolean yLatched = false;   // Y must be let go before the next drive (else it re-drives on arrival)
        boolean intakeOn = false;
        double rpmSet = Double.NaN;   // NaN: not set yet (A uses the suggestion)
        int volley = 0;
        String status = "";
        List<String> history = new ArrayList<>();
        ShotCoverage.Spot selected = null;
        Pose driveGoal = null;
        // Relocalize
        boolean relocalizing = false, relocalizeDone = false;
        ElapsedTime relocalizeTimer = new ElapsedTime();
        WallFinder relocalizeWalls = new WallFinder(100);
        List<Pose> relocalizeVision = new ArrayList<>();
        // The volley being measured
        Pose shotPose = null;
        ShotCoverage.Spot shotSpot = null;
        double shotDistance = Double.NaN, shotCamDistance = Double.NaN, shotOffAxis = Double.NaN, shotTable = Double.NaN;
        double[] shotNominal = {Double.NaN, Double.NaN};
        boolean shotAudience = true;
        String shotAim = "";
        double baseline = Double.NaN;
        boolean tipped = false;
        int stepsAtStart = 0, lastSteps = 0;
        List<Long> rpmAtBalls = new ArrayList<>();
        double minRpm = Double.NaN;
        // Per ball: how long the indexer step took (start to not busy) and the time from one step to the next
        List<Long> stepMs = new ArrayList<>(), gapMs = new ArrayList<>();
        double stepStartMs = Double.NaN;
        boolean stepTimed = true;
        double shotIntervalMs = Shooter.SHOT_INTERVAL_MS;
        int stepsDone = 0;
        int made = 0, miss = 0;

        while (opModeIsActive()) {
            String event = null;
            double stickFwd = deadband(-gamepad1.left_stick_y);
            double stickLeft = deadband(-gamepad1.left_stick_x);
            double stickTurn = deadband(-gamepad1.right_stick_x);
            boolean sticksMoved = Math.max(Math.abs(stickFwd), Math.max(Math.abs(stickLeft), Math.abs(stickTurn))) > CANCEL_STICK;
            boolean up = gamepad1.dpadUpWasPressed(), down = gamepad1.dpadDownWasPressed();
            boolean left = gamepad1.dpadLeftWasPressed(), right = gamepad1.dpadRightWasPressed();
            boolean xPressed = gamepad1.xWasPressed();
            boolean aPressed = gamepad1.aWasPressed();
            boolean backPressed = gamepad1.backWasPressed();
            boolean nextSpot = gamepad1.rightStickButtonWasPressed();
            if (gamepad1.leftBumperWasPressed() | gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
            }

            Pose pose = follower.pose();
            HiveCamera.Frame frame = camera.poll();
            boolean usable = tracker.update(frame, follower);
            if (tracker.event() != null) {
                event = tracker.event();
            }
            double[] target = tracker.target();
            double distance = Math.hypot(target[0] - pose.x(), target[1] - pose.y());
            double offAxis = ShootingZone.offAxisDeg(pose.x(), pose.y(), target[0], target[1], tracker.upAudience());
            double table = Shooter.rpmFor(nomFirst(pose, tracker.upAudience(), red));
            // Spots: odometry to the nominal CELL (the camera's CELL drifts at steep angles)
            double[] nom = ShotCoverage.nominal(pose.x(), pose.y(), tracker.upAudience(), red);
            ShotCoverage.Spot here = coverage.spotAt(nom[0], nom[1]);
            ShotCoverage.Suggestion hereSuggestion = here == null ? null : coverage.suggest(here, table);
            double suggested = hereSuggestion != null ? hereSuggestion.rpm : Math.round(table / RPM_STEP) * RPM_STEP;
            // The CELL where the camera sees it (the same point the distance is measured to), so a spot
            // driven to is the spot credited; the nominal place can be inches off on a real field
            double cellX = HiveTags.cellX(tracker.upAudience(), true), cellY = HiveTags.hiveY(red);
            if (selected == null || nextSpot || coverage.covered(selected)) {
                ShotCoverage.Spot next = coverage.nextUncovered(nextSpot ? selected : null);
                if (next != selected && next != null) {
                    event = "next test spot " + next.name();
                }
                selected = next;
            }
            if (relocalizing && usable) {
                relocalizeWalls.add(frame);
                Pose seen = TagLocalizer.solvePose(frame, POSE_MIN_TAGS, POSE_MIN_SPAN_IN);
                if (seen != null) relocalizeVision.add(seen);
            }
            if (!gamepad1.a) {
                aLatched = false;
            }

            switch (state) {
                case IDLE: {
                    if (up || down || left || right || xPressed) {
                        double base = Double.isNaN(rpmSet) ? suggested : rpmSet;
                        rpmSet = xPressed ? suggested
                                : base + (up ? RPM_STEP : down ? -RPM_STEP : right ? RPM_BIG_STEP : -RPM_BIG_STEP);
                        rpmSet = Math.max(0.0, Math.min(Flywheel.MAX_RPM, rpmSet));
                        event = String.format(Locale.US, "speed %.0f RPM (suggested %.0f, table %.0f at %.1f in)", rpmSet,
                                suggested, table, distance);
                    }
                    // Re-position from the tags: Back held RELOCALIZE_MS, standing still
                    if (gamepad1.back && !relocalizing && !relocalizeDone) {
                        relocalizing = true;
                        relocalizeTimer.reset();
                        relocalizeWalls.reset();
                        relocalizeVision.clear();
                        event = "re-position from the tags: keep Back held, stand still";
                    }
                    if (!gamepad1.back) {
                        relocalizing = false;
                        relocalizeDone = false;
                    }
                    if (relocalizing && relocalizeTimer.milliseconds() >= RELOCALIZE_MS) {
                        relocalizing = false;
                        relocalizeDone = true;
                        Pose atWall = relocalizeWalls.found(MIN_SAMPLES);
                        Pose fixed = null;
                        if (atWall != null) {
                            fixed = atWall;
                        } else if (relocalizeVision.size() >= MIN_SAMPLES) {
                            Pose v = medianPose(relocalizeVision);
                            double dh = Math.toDegrees(normalize(v.heading() - pose.heading()));
                            fixed = Math.abs(dh) <= KEEP_HEADING_DEG ? v.withHeading(pose.heading()) : v;
                        }
                        if (fixed != null) {
                            follower.setPose(fixed);
                            watchdog.poseSet();
                            follower.manual(DrivePowers.zero());
                            event = "re-positioned: " + Drawing.format(pose) + " -> " + Drawing.format(fixed);
                        } else {
                            event = "re-position failed: too few tags (stand still, HIVE in view)";
                        }
                        status = event;
                    }
                    if (!gamepad1.y) {
                        yLatched = false;
                    }
                    if (gamepad1.y && !yLatched && selected != null && watchdog.ok() && !relocalizing) {
                        yLatched = true;
                        driveGoal = ShotCoverage.pose(selected, cellX, cellY, tracker.upAudience(), red);
                        double away = Math.hypot(driveGoal.x() - pose.x(), driveGoal.y() - pose.y());
                        double turn = Math.abs(normalize(driveGoal.heading() - pose.heading()));
                        if (away < ARRIVED_IN && turn < Math.toRadians(ARRIVED_DEG)) {
                            status = "already at #" + selected.number;   // a zero-length path only shakes
                            event = status;
                        } else {
                            follower.follow(routePath(follower.pose(), ShootingZone.route(follower.pose(), driveGoal, red),
                                    driveGoal, aimer));
                            driveTimer.reset();
                            state = State.DRIVE;
                            event = "driving to " + selected.name() + " at " + Drawing.format(driveGoal);
                        }
                    }
                    if (gamepad1.a && !aLatched && !relocalizing) {
                        aLatched = true;
                        if (Double.isNaN(rpmSet)) {
                            rpmSet = suggested;
                        }
                        if (!watchdog.ok()) {
                            status = "can't aim: odometry " + watchdog.state();
                            event = status;
                        } else {
                            intakeOn = false;
                            state = State.AIM;
                            event = aimer.start() + String.format(Locale.US, "; volley %d at %.0f RPM", volley + 1, rpmSet);
                        }
                    }
                    break;
                }
                case DRIVE: {
                    if (!gamepad1.y || sticksMoved || driveTimer.milliseconds() > DRIVE_TIMEOUT_MS) {
                        follower.manual(DrivePowers.zero());
                        state = State.IDLE;
                        status = "drive stopped before the spot";
                        event = status;
                    } else if (driveTimer.milliseconds() > 100 && !follower.following()) {
                        state = State.IDLE;
                        double[] atNom = ShotCoverage.nominal(follower.pose().x(), follower.pose().y(),
                                tracker.upAudience(), red);
                        ShotCoverage.Spot at = coverage.spotAt(atNom[0], atNom[1]);
                        rpmSet = at != null ? coverage.suggest(at, table).rpm : suggested;
                        status = "at " + (at != null ? at.name() : "the spot");
                        event = String.format(Locale.US, "%s; speed %.0f RPM", status, rpmSet);
                    }
                    break;
                }
                case AIM: {
                    Aimer.Result r = aimer.update();
                    if (aimer.event() != null) event = aimer.event();
                    if (!gamepad1.a || sticksMoved) {
                        follower.manual(DrivePowers.zero());
                        state = State.IDLE;
                        status = "aim cancelled";
                        event = status;
                    } else if (r == Aimer.Result.FAILED) {
                        follower.manual(DrivePowers.zero());
                        state = State.IDLE;
                        status = "aim failed: " + aimer.status();
                        event = status;
                    } else if (r == Aimer.Result.AIMED) {
                        // Measure where the volley is shot from, then fire
                        Pose p = follower.pose();
                        double[] t = tracker.target();
                        shotPose = p;
                        shotAudience = tracker.upAudience();
                        shotDistance = Math.hypot(t[0] - p.x(), t[1] - p.y());
                        VisionAim va = VisionAim.solve(frame == null ? camera.last() : frame, red, shotAudience, p.heading());
                        shotCamDistance = va == null ? Double.NaN : va.distance;
                        shotOffAxis = ShootingZone.offAxisDeg(p.x(), p.y(), t[0], t[1], shotAudience);
                        shotNominal = ShotCoverage.nominal(p.x(), p.y(), shotAudience, red);
                        shotSpot = coverage.spotAt(shotNominal[0], shotNominal[1]);
                        shotTable = Shooter.rpmFor(shotNominal[0]);
                        shotAim = aimer.status();
                        baseline = tracker.upHeight();
                        tracker.tipping(Double.NaN);
                        tipped = false;
                        rpmAtBalls.clear();
                        minRpm = Double.NaN;
                        stepMs.clear();
                        gapMs.clear();
                        stepStartMs = Double.NaN;
                        stepTimed = true;
                        shotIntervalMs = Shooter.SHOT_INTERVAL_MS;
                        stepsAtStart = shooter.indexer().stepCount();
                        lastSteps = stepsAtStart;
                        shooter.startVolley(BALLS + SPARE_STEPS);
                        state = State.FIRE;
                        event = String.format(Locale.US, "volley %d: %.1f in (camera %s), %.0f deg off axis, %s CELL, "
                                        + "%.0f RPM (table %.0f), spot %s", volley + 1, shotDistance,
                                Double.isNaN(shotCamDistance) ? "-" : String.format(Locale.US, "%.1f", shotCamDistance),
                                shotOffAxis, tracker.upCellName(), rpmSet, shotTable,
                                shotSpot == null ? "none" : shotSpot.name());
                    }
                    break;
                }
                case FIRE: {
                    if (shooter.firing()) {
                        shooter.setPaused(!aimer.keepAim());
                        if (aimer.event() != null) event = aimer.event();
                    }
                    int steps = shooter.indexer().stepCount();
                    double nowMs = clock.milliseconds();
                    if (steps != lastSteps) {
                        lastSteps = steps;
                        rpmAtBalls.add(Math.round(flywheel.rpm()));
                        if (!Double.isNaN(stepStartMs)) gapMs.add(Math.round(nowMs - stepStartMs));
                        stepStartMs = nowMs;
                        stepTimed = false;
                    }
                    // busy() needs a moment to come on after a new target (as in Shooter)
                    if (!stepTimed && nowMs - stepStartMs > 50 && !shooter.indexer().busy()) {
                        stepMs.add(Math.round(nowMs - stepStartMs));
                        stepTimed = true;
                    }
                    if (steps != stepsAtStart) {
                        minRpm = Double.isNaN(minRpm) ? flywheel.rpm() : Math.min(minRpm, flywheel.rpm());
                    }
                    if (STOP_ON_TIP && shooter.firing() && tracker.tipping(baseline)) {
                        shooter.cancel();
                        tipped = true;
                        tracker.startTip(!shotAudience);
                        event = String.format(Locale.US, "the HIVE is tipping (tags at %.1f, were %.1f): volley stopped",
                                tracker.upHeight(), baseline);
                    }
                    if (!gamepad1.a && shooter.firing()) {
                        shooter.cancel();
                        event = "volley cancelled: A let go (still enter the result)";
                    }
                    if (!shooter.firing()) {
                        shooter.setPaused(false);
                        follower.manual(DrivePowers.zero());
                        stepsDone = Math.abs(shooter.indexer().stepCount() - stepsAtStart);
                        made = BALLS;
                        miss = 0;
                        state = State.RESULT;
                        volley++;
                        String done = shooter.stuck() ? "indexer STUCK" : tipped ? "stopped: HIVE tipping" : "done";
                        event = (event == null ? "" : event + "; ") + String.format(Locale.US,
                                "volley %d %s: %d steps; enter the result", volley, done, stepsDone);
                    }
                    break;
                }
                case RESULT: {
                    if (up) made = Math.min(BALLS, made + 1);
                    if (down) made = Math.max(0, made - 1);
                    if (xPressed) miss = (miss + 1) % MISSES.length;
                    if (aPressed) {
                        results.row(volley, clock.seconds(), red ? "red" : "blue", shotAudience ? "audience" : "far",
                                shotPose.x(), shotPose.y(), Math.toDegrees(shotPose.heading()), shotDistance,
                                Double.isNaN(shotCamDistance) ? null : shotCamDistance, shotOffAxis,
                                shotNominal[0], shotNominal[1], shotSpot == null ? null : shotSpot.number, rpmSet, shotTable,
                                Shooter.fireToleranceRpm(shotNominal[0]), join(rpmAtBalls),
                                Double.isNaN(minRpm) ? null : Math.round(minRpm), shotIntervalMs, join(stepMs), join(gapMs), stepsDone, tipped, made, BALLS,
                                MISSES[miss], flywheel.voltage(), flywheel.effectiveF(), shotAim);
                        results.flush();
                        if (stepsDone > 0) {
                            coverage.add(shotNominal[0], shotNominal[1], rpmSet, made, BALLS);
                        }
                        String line = String.format(Locale.US, "#%d %.0fin %.0fdeg %.0f: %d/%d %s", volley,
                                shotDistance, shotOffAxis, rpmSet, made, BALLS, miss == 0 ? "" : MISSES[miss])
                                + (shotIntervalMs != 250 ? String.format(Locale.US, " %.0fms", shotIntervalMs) : "");
                        history.add(0, line);
                        while (history.size() > 3) history.remove(history.size() - 1);
                        if (shotSpot != null) {
                            ShotCoverage.Suggestion s = coverage.suggest(shotSpot, shotTable);
                            rpmSet = s.rpm;
                            status = s.covered ? "#" + shotSpot.number + " COVERED: hold Y for the next" : "";
                        } else {
                            status = "saved";
                        }
                        event = "saved " + line + "; " + status;
                        state = State.IDLE;
                        aLatched = true;   // A must be let go before the next volley
                    } else if (backPressed) {
                        event = "volley " + volley + " discarded";
                        status = event;
                        state = State.IDLE;
                        relocalizeDone = true;   // this Back press is not a re-position
                    }
                    break;
                }
                default:
                    break;
            }

            // ---- Drive (driving to a spot, aiming and firing: the follower drives itself) ----
            if (state == State.IDLE || state == State.RESULT) {
                if (!watchdog.ok()) {
                    follower.manual(DrivePowers.zero());
                } else {
                    DrivePowers powers = ManualDrive.fieldCentric(stickFwd * DRIVE_SCALE, stickLeft * DRIVE_SCALE,
                            stickTurn * TURN_SCALE, pose.heading(), -driverFacing());
                    ManualDrive.driveOrHold(follower, powers);
                }
            }
            follower.update();
            watchdog.check(follower);
            pose = follower.pose();

            // ---- Shooter and intakes ----
            shooter.setRpmOverride(Double.isNaN(rpmSet) ? suggested : rpmSet);
            shooter.setDistance(state == State.FIRE ? shotNominal[0] : nom[0]);   // also the fire tolerance
            shooter.setSpin(state == State.AIM);
            String shooterEvent = shooter.update();
            if (shooterEvent != null) {
                event = event == null ? shooterEvent : event + "; " + shooterEvent;
            }
            intake.set(gamepad1.b && state != State.RESULT ? -Intake.POWER : intakeOn ? Intake.POWER : 0.0);

            // ---- Telemetry: what to do now first ----
            // Short lines: the Driver Station wraps anything much over 40 characters
            telemetry.addLine(headline(state, volley, made, miss, here, selected));
            if (state == State.RESULT) {
                telemetry.addLine("dpad: # in | X: miss | A: save | Back: drop");
            }
            if (!status.isEmpty() && state == State.IDLE) {
                telemetry.addLine(status);
            }
            if (!watchdog.ok()) {
                telemetry.addLine("!!! ODOMETRY " + watchdog.state());
            }
            if (tracker.msSinceSeen() > NO_TAGS_WARN_MS && state != State.DRIVE) {
                telemetry.addLine("!!! NO HIVE TAGS: distance is a guess");
            }
            telemetry.addData("Speed", "%.0f (suggest %.0f)", Double.isNaN(rpmSet) ? suggested : rpmSet, suggested);
            telemetry.addData("Why", hereSuggestion == null ? "table speed" : hereSuggestion.why);
            telemetry.addData("Here", "%.1fin %.0fdeg %s", nom[0], nom[1], here == null ? "(no spot)" : here.name());
            telemetry.addData("Camera", "%.1fin %.0fdeg %s CELL", distance, offAxis, tracker.upCellName());
            if (here != null) {
                telemetry.addData("Done", here.speeds());
            }
            if (selected != null) {
                Pose goal = ShotCoverage.pose(selected, cellX, cellY, tracker.upAudience(), red);
                telemetry.addData("Next", "%s, %.0fin away", selected.name(),
                        Math.hypot(goal.x() - pose.x(), goal.y() - pose.y()));
                telemetry.addData("Left", "%d of %d spots", coverage.uncoveredCount(), coverage.spotCount());
            } else {
                telemetry.addData("Next", "ALL SPOTS COVERED");
            }
            if (state == State.AIM || state == State.FIRE) {
                telemetry.addData("Flywheel", "%.0f / %.0f (+-%.0f)", flywheel.rpm(), flywheel.targetRpm(),
                        Shooter.fireToleranceRpm(state == State.FIRE ? shotNominal[0] : nom[0]));
            }
            for (String h : history) {
                telemetry.addLine(h);
            }
            if (state == State.IDLE) {
                telemetry.addLine("A shoot | Y go | RS skip | X suggest");
                telemetry.addLine("dpad speed | bumpers intake | B rev");
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            Drawing.drawTags(packet.fieldOverlay(), pose, frame);
            Drawing.drawTarget(packet.fieldOverlay(), pose, target[0], target[1], "orange");
            if (selected != null) {
                Drawing.drawRobotOutline(packet.fieldOverlay(),
                        ShotCoverage.pose(selected, cellX, cellY, tracker.upAudience(), red), "orange", selected.name());
            }
            dashboard.sendTelemetryPacket(packet);

            log.row(clock.seconds(), state, pose.x(), pose.y(), Math.toDegrees(pose.heading()), tracker.upCellName(),
                    distance, offAxis, here == null ? null : here.number, Double.isNaN(rpmSet) ? null : rpmSet,
                    flywheel.rpm(), flywheel.mode(), flywheel.isOn() ? flywheel.amps() : null, flywheel.voltage(),
                    shooter.indexer().stepCount(), shooter.indexer().degrees(), shooter.indexer().targetDegrees(),
                    shooter.indexer().state(), frame == null ? null : frame.ids(), tracker.hiveStateName(),
                    Double.isNaN(tracker.upHeight()) ? null : tracker.upHeight(), event);
        }
        shooter.stop();
        intake.off();
        camera.stop();
        log.close();
        results.close();
        watchdog.close();
    }

    /** The first telemetry line: what is happening and what to do. */
    private static String headline(State state, int volley, int made, int miss, ShotCoverage.Spot here,
                                   ShotCoverage.Spot selected) {
        switch (state) {
            case DRIVE:
                return ">>> DRIVING to " + (selected == null ? "spot" : "#" + selected.number) + " (let go: stop)";
            case AIM:
                return ">>> AIMING (let go of A: cancel)";
            case FIRE:
                return ">>> FIRING";
            case RESULT:
                return String.format(Locale.US, ">>> VOLLEY %d: %d/%d IN, %s", volley, made, BALLS, MISSES[miss]);
            default:
                return here == null ? ">>> NOT AT A SPOT: hold Y"
                        : String.format(Locale.US, ">>> AT #%d: load %d, hold A", here.number, BALLS);
        }
    }

    /** The route as one path, the shooter turning toward the CELL on the way (it arrives aimed). */
    private static Path routePath(Pose start, List<double[]> route, Pose goal, Aimer aimer) {
        Path[] legs = new Path[route.size() - 1];
        for (int i = 0; i < legs.length; i++) {
            Pose a = new Pose(route.get(i)[0], route.get(i)[1], start.heading());
            Pose b = new Pose(route.get(i + 1)[0], route.get(i + 1)[1], goal.heading());
            legs[i] = line(a, b).heading(aimer.facingTarget(i == 0 ? start.heading() : null));
        }
        return legs.length == 1 ? legs[0] : path(legs);
    }

    /** Shooting distance as the matches use it: odometry to the nominal up CELL. */
    private static double nomFirst(Pose pose, boolean audience, boolean red) {
        return ShootingZone.shotDistance(pose.x(), pose.y(), audience, red);
    }

    private Pose startPose(WallFinder walls, List<Pose> vision, AutoSettings settings) {
        Pose wall = walls.found(MIN_SAMPLES);
        if (wall != null) return wall;
        if (vision.size() >= MIN_SAMPLES) return medianPose(vision);
        return AutoSettings.startPose(red ? AutoSettings.Alliance.RED : AutoSettings.Alliance.BLUE, settings.tile());
    }

    private String startSource(WallFinder walls, List<Pose> vision) {
        return walls.found(MIN_SAMPLES) != null ? walls.describe(MIN_SAMPLES)
                : vision.size() >= MIN_SAMPLES ? "vision pose, " + vision.size() + " frames"
                : "the Game Setup start wall (no tag fix)";
    }

    private static String join(List<Long> values) {
        StringBuilder sb = new StringBuilder();
        for (Long v : values) {
            if (sb.length() > 0) sb.append(' ');
            sb.append(v);
        }
        return sb.toString();
    }

    private static double median(List<Double> v) {
        List<Double> s = new ArrayList<>(v);
        Collections.sort(s);
        int n = s.size();
        return n == 0 ? Double.NaN : n % 2 == 1 ? s.get(n / 2) : (s.get(n / 2 - 1) + s.get(n / 2)) / 2.0;
    }

    private static Pose medianPose(List<Pose> poses) {
        List<Double> xs = new ArrayList<>(), ys = new ArrayList<>(), hs = new ArrayList<>();
        double ref = poses.get(0).heading();
        for (Pose p : poses) {
            xs.add(p.x());
            ys.add(p.y());
            hs.add(normalize(p.heading() - ref));
        }
        return new Pose(median(xs), median(ys), normalize(ref + median(hs)));
    }

    /** Field direction the driver looks along (FTC, radians): red +90 deg, blue -90 deg. */
    private double driverFacing() {
        return Math.toRadians(red ? 90.0 : -90.0);
    }

    private static double deadband(double v) {
        return Math.abs(v) < STICK_DEADBAND ? 0.0 : v;
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
