package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
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
import org.firstinspires.ftc.teamcode.John.Vision.TagLocalizer;
import org.firstinspires.ftc.teamcode.John.Vision.VisionAim;
import org.firstinspires.ftc.teamcode.John.Vision.WallFinder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Shot calibration: finds the flywheel speed for each distance (and angle) to the up CELL, for
 * Shooter.RPM_DISTANCES_IN / RPM_VALUES. Drive anywhere, set the speed by hand, shoot BALLS balls,
 * then enter how many went in. Every volley is one row of the results log.
 *
 * START: as in the TeleOp, from the tags in INIT: flush on a back wall (WallFinder) is exact, else
 * the vision pose (TagLocalizer.solvePose), else the Game Setup start wall. The distance does not
 * need an exact position: the CELL is placed by the camera from the same odometry pose (CellTracker),
 * so an offset cancels out; the heading matters (the angle off the CELL axis). Back held 1 s: re-
 * position from the tags. The alliance (our HIVE) comes from Game Setup, else B / X in INIT.
 *
 * One volley:
 *  1. Load BALLS balls (bumpers: intakes on / off; B held: reverse).
 *  2. Set the speed: dpad up / down +/- RPM_STEP, dpad left / right -/+ RPM_BIG_STEP, X: the table's
 *     speed for the current distance. The telemetry shows both.
 *  3. A held: aims at the up CELL (Aimer, camera-refined) from where John stands (no drive into
 *     range: any spot can be tested; the zone check is only shown), then fires BALLS + SPARE_STEPS
 *     indexer steps at the set speed. Let go of A: cancel. If the HIVE tips during the volley
 *     (CellTracker.tipping) the volley stops: the rest would go at a moving CELL.
 *  4. RESULT: dpad up / down how many went in (0..BALLS), X the miss type (none / short / long /
 *     left / right / bounced out / mixed), A saves, Back discards. Then the next volley.
 * Each saved result: the distance (odometry + camera CELL) and the camera-only distance (VisionAim)
 * when the volley started, the angle off the CELL axis, which CELL, the set and table speeds, the
 * speed at each ball, the balls seen (flywheel dips of more than DIP_RPM), battery and F.
 *
 * The HIVE tips after a few balls in the up CELL; the camera then finds the other CELL up, and the
 * next volley goes there (a different distance and angle from the same spot). Empty the CELLS when
 * needed.
 *
 * Logged to /sdcard/FIRST/john_logs/r<run>_john_shot_results_<time>.csv (one row per volley) and
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
    /** A flywheel dip deeper than this below the set speed counts as a ball fired. */
    public static double DIP_RPM = 100;
    public static boolean STOP_ON_TIP = true;
    public static double DRIVE_SCALE = 0.6;
    public static double TURN_SCALE = 0.5;
    public static double STICK_DEADBAND = 0.05;
    public static double CANCEL_STICK = 0.3;
    public static int START_SAMPLES = 60;
    public static int MIN_SAMPLES = 5;
    public static double RELOCALIZE_MS = 1000;
    /** Re-position: keep the Pinpoint heading when the vision heading is within this of it. */
    public static double KEEP_HEADING_DEG = 10.0;
    public static int POSE_MIN_TAGS = 3;
    public static double POSE_MIN_SPAN_IN = 8.0;

    private static final String[] MISSES = {"none", "short", "long", "left", "right", "bounced out", "mixed"};

    private enum State { IDLE, AIM, FIRE, RESULT }

    private static final String LOG_HEADER = "t_s,state,x,y,h_deg,cell,distance,cam_distance,off_axis_deg,"
            + "rpm_set,table_rpm,fw_rpm,fw_mode,fw_amps,battery_v,idx_steps,tags,hive,up_height_in,event";
    private static final String RESULT_HEADER = "volley,t_s,alliance,cell,x,y,h_deg,distance_in,cam_distance_in,"
            + "off_axis_deg,rpm_set,table_rpm,rpm_at_balls,min_rpm,balls_seen,steps_done,tipped,made,balls,miss,"
            + "battery_v,fw_f,aim";

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
            Pose start = startPose(walls, initVision, settings);
            telemetry.addData("Alliance", "%s HIVE%s", red ? "RED" : "BLUE", haveSetup ? " (Game Setup)" : " (B red, X blue)");
            telemetry.addData("Start pose (FTC)", "%s  from %s", Drawing.format(start), startSource(walls, initVision));
            if (walls.found(MIN_SAMPLES) == null && initVision.size() < MIN_SAMPLES) {
                telemetry.addLine("!!! No tag fix: put the camera wall flush on a back wall facing the HIVE, "
                        + "or hold Back after START to re-position from the tags.");
            }
            telemetry.addData("Camera", camera.status());
            telemetry.addData("Logs", "%s, %s", results.fileName(), log.fileName());
            telemetry.addLine(String.format(Locale.US, "Load %d balls; A held aims and fires; then enter the result.", BALLS));
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
        log.row(0.0, State.IDLE, start.x(), start.y(), Math.toDegrees(start.heading()), null, null, null, null,
                null, null, null, null, null, flywheel.voltage(), null, null, null, null,
                "start " + (red ? "red" : "blue") + " at " + Drawing.format(start) + " (" + startSource(walls, initVision) + ")");

        State state = State.IDLE;
        boolean aLatched = false;
        boolean intakeOn = false;
        double rpmSet = Double.NaN;   // NaN: not set yet, X or the dpad sets it
        int volley = 0;
        String status = "";
        List<String> history = new ArrayList<>();
        // Relocalize
        boolean relocalizing = false, relocalizeDone = false;
        ElapsedTime relocalizeTimer = new ElapsedTime();
        WallFinder relocalizeWalls = new WallFinder(100);
        List<Pose> relocalizeVision = new ArrayList<>();
        // The volley being measured
        Pose shotPose = null;
        double shotDistance = Double.NaN, shotCamDistance = Double.NaN, shotOffAxis = Double.NaN, shotTable = Double.NaN;
        boolean shotAudience = true;
        String shotAim = "";
        double baseline = Double.NaN;
        boolean tipped = false;
        int stepsAtStart = 0, lastSteps = 0;
        List<Long> rpmAtBalls = new ArrayList<>();
        double minRpm = Double.NaN;
        int ballsSeen = 0;
        boolean inDip = false;
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
            double table = Shooter.rpmFor(distance);
            if (relocalizing && usable) {
                relocalizeWalls.add(frame);
                Pose seen = TagLocalizer.solvePose(frame, POSE_MIN_TAGS, POSE_MIN_SPAN_IN);
                if (seen != null) relocalizeVision.add(seen);
            }

            switch (state) {
                case IDLE: {
                    if (up || down || left || right || xPressed) {
                        double base = Double.isNaN(rpmSet) ? Math.round(table / RPM_STEP) * RPM_STEP : rpmSet;
                        rpmSet = xPressed ? Math.round(table / RPM_STEP) * RPM_STEP
                                : base + (up ? RPM_STEP : down ? -RPM_STEP : right ? RPM_BIG_STEP : left ? -RPM_BIG_STEP : 0.0);
                        rpmSet = Math.max(0.0, Math.min(Flywheel.MAX_RPM, rpmSet));
                        event = String.format(Locale.US, "speed set to %.0f RPM (table %.0f at %.1f in)", rpmSet, table, distance);
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
                    }
                    if (!gamepad1.a) {
                        aLatched = false;
                    }
                    if (gamepad1.a && !aLatched && !relocalizing) {
                        aLatched = true;
                        if (Double.isNaN(rpmSet)) {
                            rpmSet = Math.round(table / RPM_STEP) * RPM_STEP;
                        }
                        if (!watchdog.ok()) {
                            status = "can't aim: odometry " + watchdog.state();
                            event = status;
                        } else {
                            intakeOn = false;
                            state = State.AIM;
                            event = aimer.start() + String.format(Locale.US, "; volley %d at %.0f RPM", volley + 1, rpmSet);
                            status = "aiming";
                        }
                    }
                    break;
                }
                case AIM: {
                    Aimer.Result r = aimer.update();
                    status = aimer.status();
                    if (aimer.event() != null) event = aimer.event();
                    if (!gamepad1.a || sticksMoved) {
                        follower.manual(DrivePowers.zero());
                        state = State.IDLE;
                        status = "cancelled";
                        event = "aim cancelled";
                    } else if (r == Aimer.Result.FAILED) {
                        follower.manual(DrivePowers.zero());
                        state = State.IDLE;
                        event = "aim failed: " + aimer.status();
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
                        shotTable = Shooter.rpmFor(shotDistance);
                        shotAim = aimer.status();
                        baseline = tracker.upHeight();
                        tracker.tipping(Double.NaN);
                        tipped = false;
                        rpmAtBalls.clear();
                        minRpm = Double.NaN;
                        ballsSeen = 0;
                        inDip = false;
                        stepsAtStart = shooter.indexer().stepCount();
                        lastSteps = stepsAtStart;
                        shooter.startVolley(BALLS + SPARE_STEPS);
                        state = State.FIRE;
                        event = String.format(Locale.US, "volley %d: %.1f in (camera %s), %.0f deg off axis, %s CELL, "
                                        + "%.0f RPM (table %.0f)", volley + 1, shotDistance,
                                Double.isNaN(shotCamDistance) ? "-" : String.format(Locale.US, "%.1f", shotCamDistance),
                                shotOffAxis, tracker.upCellName(), rpmSet, shotTable);
                    }
                    break;
                }
                case FIRE: {
                    if (shooter.firing()) {
                        shooter.setPaused(!aimer.keepAim());
                        if (aimer.event() != null) event = aimer.event();
                    }
                    int steps = shooter.indexer().stepCount();
                    if (steps != lastSteps) {
                        lastSteps = steps;
                        rpmAtBalls.add(Math.round(flywheel.rpm()));
                    }
                    if (steps != stepsAtStart) {
                        double rpm = flywheel.rpm();
                        minRpm = Double.isNaN(minRpm) ? rpm : Math.min(minRpm, rpm);
                        if (!inDip && rpm < rpmSet - DIP_RPM) {
                            inDip = true;
                            ballsSeen++;
                        } else if (inDip && rpm > rpmSet - DIP_RPM / 2.0) {
                            inDip = false;
                        }
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
                        made = Math.min(BALLS, ballsSeen);   // a guess to start from: the balls seen
                        miss = 0;
                        state = State.RESULT;
                        volley++;
                        String done = shooter.stuck() ? "indexer STUCK" : tipped ? "stopped: HIVE tipping" : "done";
                        event = (event == null ? "" : event + "; ") + String.format(Locale.US,
                                "volley %d %s: %d steps, %d balls seen; enter the result", volley, done, stepsDone, ballsSeen);
                    }
                    break;
                }
                case RESULT: {
                    if (up) made = Math.min(BALLS, made + 1);
                    if (down) made = Math.max(0, made - 1);
                    if (xPressed) miss = (miss + 1) % MISSES.length;
                    if (aPressed) {
                        double v = flywheel.voltage();
                        results.row(volley, clock.seconds(), red ? "red" : "blue", shotAudience ? "audience" : "far",
                                shotPose.x(), shotPose.y(), Math.toDegrees(shotPose.heading()), shotDistance,
                                Double.isNaN(shotCamDistance) ? null : shotCamDistance, shotOffAxis, rpmSet, shotTable,
                                join(rpmAtBalls), Double.isNaN(minRpm) ? null : Math.round(minRpm), ballsSeen, stepsDone,
                                tipped, made, BALLS, MISSES[miss], v, flywheel.effectiveF(), shotAim);
                        results.flush();
                        String line = String.format(Locale.US, "#%d %.1f in %.0f deg %s: %.0f RPM -> %d/%d %s%s", volley,
                                shotDistance, shotOffAxis, shotAudience ? "aud" : "far", rpmSet, made, BALLS,
                                MISSES[miss], tipped ? " (tipped)" : "");
                        history.add(0, line);
                        while (history.size() > 8) history.remove(history.size() - 1);
                        event = "saved " + line;
                        state = State.IDLE;
                        aLatched = true;   // A must be let go before the next volley
                    } else if (backPressed) {
                        event = "volley " + volley + " discarded";
                        state = State.IDLE;
                        relocalizeDone = true;   // this Back press is not a re-position
                    }
                    break;
                }
                default:
                    break;
            }

            // ---- Drive (aim / fire hold the pose themselves) ----
            boolean degraded = !watchdog.ok();
            if (state == State.AIM || state == State.FIRE) {
                // the follower holds the aim pose
            } else if (degraded) {
                follower.manual(DrivePowers.zero());
            } else {
                DrivePowers powers = ManualDrive.fieldCentric(stickFwd * DRIVE_SCALE, stickLeft * DRIVE_SCALE,
                        stickTurn * TURN_SCALE, pose.heading(), -driverFacing());
                ManualDrive.driveOrHold(follower, powers);
            }
            follower.update();
            watchdog.check(follower);
            pose = follower.pose();

            // ---- Shooter and intakes ----
            if (!Double.isNaN(rpmSet)) {
                flywheel.setTargetRpm(rpmSet);
            }
            shooter.setSpin(state == State.AIM);
            String shooterEvent = shooter.update();
            if (shooterEvent != null) {
                event = event == null ? shooterEvent : event + "; " + shooterEvent;
            }
            intake.set(gamepad1.b && state != State.RESULT ? -Intake.POWER : intakeOn ? Intake.POWER : 0.0);

            // ---- Telemetry ----
            if (state == State.RESULT) {
                telemetry.addLine(String.format(Locale.US, ">>> VOLLEY %d RESULT: %d of %d in, miss: %s", volley, made, BALLS,
                        MISSES[miss]));
                telemetry.addLine("    dpad up / down: how many went in;  X: miss type;  A: save;  Back: discard");
                telemetry.addData("Volley", "%.1f in (camera %s), %.0f deg off axis, %s CELL, %.0f RPM; %d steps, "
                                + "%d balls seen (dips)%s", shotDistance,
                        Double.isNaN(shotCamDistance) ? "-" : String.format(Locale.US, "%.1f", shotCamDistance), shotOffAxis,
                        shotAudience ? "audience" : "far", rpmSet, stepsDone, ballsSeen, tipped ? ", HIVE TIPPED" : "");
            } else {
                telemetry.addData("Speed", "%s   (dpad +/-%.0f, left/right +/-%.0f, X table)",
                        Double.isNaN(rpmSet) ? "not set (A uses the table)" : String.format(Locale.US, "%.0f RPM", rpmSet),
                        RPM_STEP, RPM_BIG_STEP);
                telemetry.addData("Table", "%.0f RPM at %.1f in%s", table, distance,
                        Shooter.rpmInRange(distance) ? "" : " (outside the tested distances)");
                telemetry.addData("State (hold A)", "%s%s", state, status.isEmpty() ? "" : ": " + status);
            }
            String zone = ShootingZone.problem(pose.x(), pose.y(), target[0], target[1], tracker.upAudience());
            telemetry.addData("Target", "%s CELL, %.1f in, %.0f deg off axis (%s)%s", tracker.upCellName(), distance,
                    ShootingZone.offAxisDeg(pose.x(), pose.y(), target[0], target[1], tracker.upAudience()),
                    tracker.source(), zone == null ? "" : "; outside the match zone: " + zone);
            telemetry.addData("Flywheel", "%.0f / %.0f RPM %s, %.1f V", flywheel.rpm(), flywheel.targetRpm(),
                    flywheel.atSpeed() ? "AT SPEED" : "", flywheel.voltage());
            telemetry.addData("Intakes (bumpers, B reverse)", intake.status());
            telemetry.addData("Pose (FTC)", "%s  %s", Drawing.format(pose), watchdog.state());
            telemetry.addData("HIVE", "%s, up tags at %s in", tracker.hiveStateName(),
                    Double.isNaN(tracker.upHeight()) ? "-" : String.format(Locale.US, "%.1f", tracker.upHeight()));
            for (String h : history) {
                telemetry.addLine(h);
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            Drawing.drawTags(packet.fieldOverlay(), pose, frame);
            Drawing.drawTarget(packet.fieldOverlay(), pose, target[0], target[1], "orange");
            dashboard.sendTelemetryPacket(packet);

            log.row(clock.seconds(), state, pose.x(), pose.y(), Math.toDegrees(pose.heading()), tracker.upCellName(),
                    distance, null, ShootingZone.offAxisDeg(pose.x(), pose.y(), target[0], target[1], tracker.upAudience()),
                    Double.isNaN(rpmSet) ? null : rpmSet, table, flywheel.rpm(), flywheel.mode(),
                    flywheel.isOn() ? flywheel.amps() : null, flywheel.voltage(), shooter.indexer().stepCount(),
                    frame == null ? null : frame.ids(), tracker.hiveStateName(),
                    Double.isNaN(tracker.upHeight()) ? null : tracker.upHeight(), event);
        }
        shooter.stop();
        intake.off();
        camera.stop();
        log.close();
        results.close();
        watchdog.close();
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
