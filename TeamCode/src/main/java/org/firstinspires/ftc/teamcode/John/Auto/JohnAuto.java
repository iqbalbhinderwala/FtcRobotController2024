package org.firstinspires.ftc.teamcode.John.Auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Aimer;
import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Hardware.Intake;
import org.firstinspires.ftc.teamcode.John.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.Pedro.PathWatcher;
import org.firstinspires.ftc.teamcode.John.RobotGeometry;
import org.firstinspires.ftc.teamcode.John.ShootingZone;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;
import org.firstinspires.ftc.teamcode.John.Vision.CellTracker;
import org.firstinspires.ftc.teamcode.John.Vision.HiveCamera;
import org.firstinspires.ftc.teamcode.John.Vision.HiveTags;
import org.firstinspires.ftc.teamcode.John.Vision.TagLocalizer;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * John's autonomous. The alliance, start tile and start delay come from [John] Game Setup (run it
 * before the match; see AutoSettings). INIT shows them, finds the start position along the wall
 * with the camera and previews it on the Dashboard field; no buttons are used, so nothing can be
 * changed while it waits. Without a saved Game Setup it refuses to run: the robot does not move.
 *
 * START POSITION: camera wall flush on the audience wall (red; blue: the far wall), shooter facing
 * the field, on the Game Setup tile. During INIT the camera reads the HIVE tags and measures where
 * John is along the wall (TagLocalizer, as in the TeleOp); that Y is used when it is steady
 * (spread <= START_MAX_SPREAD_IN), else the tile's.
 *
 * Routine (written for RED in FTC field coordinates, inches; BLUE is reflected through the center):
 *  1. Step straight off the wall just far enough for the corner to clear it, turning toward the up
 *     CELL on the way as far as the corner allows (one path), so it arrives at the aim heading.
 *  2. Aim at the up CELL (Aimer: refine with the camera) and shoot the 4 preloaded balls (Shooter:
 *     flywheel speed from the distance, one ball per indexer step, jam handling).
 *  3. GARDEN: turn back, slide flush on the audience wall, then drive along it toward the red corner
 *     with the intakes on: fast until just before the first ball, then COLLECT_SPEED_IPS. The intake
 *     keeps running INTAKE_AFTER_GARDEN_S longer (the balls move on toward the indexer while driving),
 *     unless the next volley or the park comes first.
 *  4. Look at the HIVE from near the GARDEN (LOOK_X, LOOK_Y: B1, camera facing the HIVE). All of a
 *     HIVE's tags face the end that is up, so from our side they are readable exactly when the near
 *     CELL is up; once it has tipped they all face away. Seen: the camera confirms which CELL is up
 *     (CellTracker). Not seen within LOOK_TIMEOUT_MS (camera working): with NOT_SEEN_MEANS_TIPPED
 *     the far CELL is up (it says so in the log; a robot in the way would look the same), else no
 *     second volley. The look spot is in the near CELL's shooting zone: if that one is up, John
 *     shoots from there.
 *  5. Go to the up CELL's side: the nearest spot in its shooting zone (ShootingZone, around the
 *     HIVE frame): audience CELL up, the audience side; down, the far side. On the way the shooter
 *     keeps turning toward the CELL (Aimer.facingTarget), so it arrives aimed and the Aimer only
 *     refines. Shoot again.
 * During a volley (Aimer.keepAim): knocked off aim, it holds fire until the hold has brought it
 * back; pushed away more than Aimer.HOLD_SHIFT_IN, it aims again from there.
 *  6. PARK in the LOADING ZONE, around the HIVE frame; the last stretch at PARK_SPEED_IPS.
 * Tipping during a volley (both volleys): when the camera sees the target CELL's tags drop
 * (CellTracker.tipping) it stops shooting, goes to the other side (the other CELL is now up), aims
 * and shoots the rest (at most MAX_SIDE_SWITCHES times per volley).
 * A failed aim (more than Aimer.AIM_MAX_ERROR_DEG off) skips that volley.
 *
 * At most 4 balls at a time (G407): there is no ball sensor, so a volley counts as having emptied
 * John only when all its indexer steps were done (not skipped, stuck or cut short by a tip). If
 * volley 1 did not, it is tried once more; if it still did not, the GARDEN is skipped (he may still
 * hold balls) and he goes on to look and shoot with what he has.
 *
 * Always time to park: every loop, if the time left (AUTO_S) is no more than the estimated park
 * time from where John is (parkSeconds) plus PARK_MARGIN_S, whatever runs is dropped and he parks.
 * And before each stage (the volley 1 retry, GARDEN, look, shot 2, a side switch) its estimated
 * time plus the park from where it ends must fit, else it and the rest are skipped. Estimates:
 * AVG_DRIVE_IPS along the route (around the HIVE) plus DRIVE_OVERHEAD_S per drive, AIM_S, VOLLEY_S.
 * The park deadline runs in STEP_MODE too; there the time spent waiting for Y does not count
 * (autoSeconds()), and a forced park does not wait for Y. In STEP_MODE the stage guards only log
 * their estimates. For step-mode tests switch the Driver Station's 30 s timer off (the timer next to
 * INIT), or it stops the OpMode at 30 s of real time.
 *
 * Nothing may hang: every stage has a hard limit (*_MAX_S); past it, it is dropped (shooter
 * stopped, the robot holds where it is), logged, and the routine goes on. Every follow and creep
 * has its own timeout (PathWatcher), every volley VOLLEY_MAX_S. The pose (and the up CELL) is
 * saved to the blackboard every loop for the TeleOp.
 *
 * Other robots: every drive goes through PathWatcher. When John is BLOCKED (not moving with path
 * left) or PUSHED off the path, it stops pushing, waits, plans again from where it is and retries;
 * after PathWatcher.RETRIES it gives up and the routine goes on with the next stage. Parking (the
 * partner robot will likely park in the same LOADING ZONE): if the park is blocked, John stays if
 * he is already partly in the LOADING ZONE (that scores), otherwise he tries spots PARK_ALT_OFFSET_IN
 * to either side along it, the nearer one first.
 *
 * Slow stretches (the GARDEN pass, the last park stretch) are PathWatcher.creep: a capped-speed
 * Pedro path never got John moving (r0109 / r0111: its coast feedforward is below the static
 * friction at 12-15 in/s).
 * STEP_MODE (testing) waits for gamepad1 Y before each stage and turns the park deadline off.
 *
 * Every run is logged to /sdcard/FIRST/john_logs/r<run>_john_auto_<time>.csv: one row per loop
 * and an event row per decision. The last pose and the up CELL go to the blackboard for
 * [John] TeleOp (preselected), which starts from them.
 */
@Config("JohnAuto")
@Autonomous(name = "[John] Auto", group = "John", preselectTeleOp = "[John] TeleOp")
public class JohnAuto extends LinearOpMode {

    // ---- Red positions, FTC field coordinates (inches); blue is reflected ----
    /** Audience wall and red wall. */
    public static double AUDIENCE_WALL_X = 72.0;
    public static double RED_WALL_Y = -72.0;
    /** Gap kept from a wall when driving along it. */
    public static double WALL_GAP = 0.5;
    /** Extra room between the turning robot's corner and the wall. */
    public static double TURN_MARGIN = 0.5;
    /** GARDEN: the 4 balls lie along the audience wall; the first (farthest from the corner) at this Y. */
    public static double GARDEN_FIRST_BALL_Y = -61.9;
    public static double BALL_RADIUS = 1.4;
    /** Start the slow stretch this far before intake A reaches the first ball. */
    public static double SLOW_BEFORE_BALL_IN = 4.0;
    /** Park: partly inside the red LOADING ZONE (X -48 to -24, Y -72 to -60.4), camera wall on the wall. */
    public static double PARK_X = -36.0;
    /**
     * 0: the camera wall against the red wall, the shooter (and camera) facing the field / HIVE; the
     * last stretch slides along the wall along John's length (2026-10-24, was 90: an intake end to
     * the wall). Parked like this the camera sees the HIVE, which helps the TeleOp's start check.
     */
    public static double PARK_HEADING_DEG = 0.0;
    /** Other park spots along the LOADING ZONE, this far to either side of PARK_X, if the park is blocked. */
    public static double PARK_ALT_OFFSET_IN = 10.0;
    /** Red LOADING ZONE (FTC); PARK scores when the robot is partly inside (by at least PARK_OVERLAP_IN). */
    public static double LOADING_ZONE_MIN_X = -48.0;
    public static double LOADING_ZONE_MAX_X = -24.0;
    public static double LOADING_ZONE_MAX_Y = -60.4;
    public static double PARK_OVERLAP_IN = 1.0;
    /**
     * Before the last park stretch: just off the red wall, beside the LOADING ZONE, at the park
     * heading; the drive there goes around the HIVE frame (ShootingZone.route). The last stretch then
     * slides along the wall into the zone, so a robot already there is pushed along the wall, not
     * into it (2026-10-24: coming in at an angle it tried to push a robot into the wall).
     */
    public static double PARK_APPROACH_X = -12.0;
    /** The approach is this far off the park line (away from the wall), the rest is along the wall. */
    public static double PARK_APPROACH_OFF_WALL_IN = 1.0;

    // ---- Start position from the camera ----
    public static int START_SAMPLES = 60;
    public static int MIN_START_SAMPLES = 5;
    /** Use the camera's Y only when its readings agree this well (median distance from the median). */
    public static double START_MAX_SPREAD_IN = 2.0;
    /** After this long in INIT, warn if the camera has not seen a HIVE tag. */
    public static double CAMERA_CHECK_S = 2.0;

    // ---- Looking at the HIVE after the GARDEN ----
    /**
     * Where to look from (red frame): B1, near the GARDEN, far enough off the audience wall to turn
     * to the HIVE (camera facing its center); also in the near CELL's shooting zone (49", 28 deg).
     * Beside the HIVE (0, -44) did not work: no tag seen in 2.5 s (r0111), too oblique.
     */
    public static double LOOK_X = 56.0;
    public static double LOOK_Y = -36.0;
    /**
     * Not seeing any of our HIVE's tags from the look spot means it has tipped: from our side its
     * tags are readable only while the near CELL is up. Only if the camera has seen HIVE tags earlier
     * in the run (so it works; a camera with a bad exposure would otherwise "see" a tip).
     */
    public static boolean NOT_SEEN_MEANS_TIPPED = true;
    /**
     * Look at least this long (the HIVE may still be settling), at most LOOK_TIMEOUT_MS; it counts
     * when a frame at most CONFIRM_MAX_AGE_MS old agrees with the up CELL.
     */
    public static double LOOK_MIN_MS = 600;
    public static double LOOK_TIMEOUT_MS = 2500;
    public static double CONFIRM_MAX_AGE_MS = 200;
    /** Times per volley it may stop for a tip and go to the other side. */
    public static int MAX_SIDE_SWITCHES = 2;

    // ---- Speeds and timing ----
    public static double COLLECT_SPEED_IPS = 12.0;
    /** After the GARDEN the intake runs this much longer (the balls move on toward the indexer). */
    public static double INTAKE_AFTER_GARDEN_S = 2.0;
    public static double PARK_SPEED_IPS = 15.0;
    /** Turning back to the wall before the GARDEN. */
    public static double TURN_TOLERANCE_DEG = 1.5;
    public static double TURN_TIMEOUT_MS = 1500;
    // ---- Time: always enough left to park ----
    /** AUTO length; PARK counts at its end. */
    public static double AUTO_S = 30.0;
    /** Kept in hand on top of the park estimate. */
    public static double PARK_MARGIN_S = 1.5;
    /** Estimates: average speed along a route, plus this per drive (speeding up / slowing down). */
    public static double AVG_DRIVE_IPS = 30.0;
    public static double DRIVE_OVERHEAD_S = 0.6;
    public static double PARK_OVERHEAD_S = 1.0;
    public static double AIM_S = 1.2;
    public static double VOLLEY_S = 3.5;
    /** Hard limits per stage (seconds): past them the stage is dropped and the routine goes on. */
    public static double STEP_AIM_MAX_S = 6.0;
    public static double VOLLEY_MAX_S = 9.0;
    public static double GARDEN_MAX_S = 12.0;
    public static double LOOK_MAX_S = 9.0;
    public static double SHOT_MAX_S = 18.0;
    public static double PARK_MAX_S = 12.0;
    /** Testing: wait for gamepad1 Y before each stage; the park deadline is off. */
    public static boolean STEP_MODE = false;     // match
    // public static boolean STEP_MODE = true;   // TEST: press Y to start each stage

    private static final String LOG_HEADER = "t_s,step,x,y,h_deg,follower,shooter,intake,target_x,target_y,"
            + "target_src,up_cell,tags,watchdog,event";

    private Follower follower;
    private ForesightConfig foresight;
    private OdometryWatchdog watchdog;
    private PathWatcher watcher;
    private HiveCamera camera;
    private CellTracker tracker;
    private Aimer aimer;
    private Shooter shooter;
    private Intake intake;
    private boolean blue;
    private String step = "init";
    private boolean advance;
    private Aimer.Result aimResult = Aimer.Result.FAILED;
    /** The camera confirmed which CELL is up after the GARDEN. */
    private boolean upKnown;
    /** The last volley did all its indexer steps (John holds no balls). */
    private boolean emptied;
    /** A stage did not fit: skip the rest and park. */
    private boolean parkNow;
    private boolean routineDone;
    private HiveCamera.Frame frame;
    private final ElapsedTime clock = new ElapsedTime();
    /** Step mode: time spent waiting for Y, not counted as autonomous time. */
    private double waitedS;
    private double lastLoopS;
    /** When to switch the intake off after the GARDEN (autonomous seconds), or NaN. */
    private double intakeOffAtS = Double.NaN;
    /** The camera has seen a HIVE tag in this run (INIT included): it works, so not seeing one means something. */
    private boolean cameraSawTags;
    private CsvLog log;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        foresight = Constants.foresightConfig();
        follower = Constants.create(hardwareMap, foresight);
        watchdog = new OdometryWatchdog(hardwareMap, "auto");
        watcher = new PathWatcher(follower, this::logEvent);
        camera = new HiveCamera(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        boolean haveSettings = AutoSettings.isSaved(blackboard);
        AutoSettings settings = new AutoSettings(blackboard);
        blue = settings.alliance() == AutoSettings.Alliance.BLUE;
        tracker = new CellTracker(!blue);
        aimer = new Aimer(follower, tracker, camera.available());
        TagLocalizer startFix = new TagLocalizer(START_SAMPLES);

        // INIT: show the Game Setup, find the position along the wall (no buttons, nothing moves)
        ElapsedTime initTimer = new ElapsedTime();
        int initFrames = 0;
        int initFramesWithTags = 0;
        while (opModeInInit()) {
            Pose wall = settings.startPose();
            HiveCamera.Frame initFrame = camera.poll();
            if (initFrame != null) {
                initFrames++;
                if (!initFrame.tags.isEmpty()) {
                    initFramesWithTags++;
                    cameraSawTags = true;
                }
            }
            startFix.add(initFrame, wall.heading());
            String cameraWarning = cameraWarning(initTimer.seconds(), initFrames, initFramesWithTags);
            if (cameraWarning != null) {
                telemetry.addLine("!!! " + cameraWarning);
            }
            Pose start = startPose(wall, startFix);
            if (haveSettings) {
                telemetry.addLine("Game Setup:");
                settings.addTelemetry(telemetry, false);
                if (startFix.count() >= MIN_START_SAMPLES) {
                    telemetry.addData("Camera: along the wall", "Y %.1f (tile %.1f, %+.1f), spread %.1f, %d readings%s",
                            startFix.y(), wall.y(), startFix.y() - wall.y(), startFix.ySpread(), startFix.count(),
                            useCamera(startFix) ? ": USED" : ": too unsteady, using the tile");
                } else {
                    telemetry.addData("Camera: along the wall", "no tag fix (%d readings): using the tile", startFix.count());
                }
                telemetry.addData("Start pose (FTC)", Drawing.format(start));
                if (STEP_MODE) {
                    telemetry.addLine("STEP MODE (test): press Y to start each stage.");
                }
            } else {
                telemetry.addLine("NO GAME SETUP: this autonomous will not run.");
                telemetry.addLine("Stop, run [John] Game Setup, then INIT [John] Auto again.");
            }
            telemetry.addData("Camera", camera.status());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            if (haveSettings) {
                Drawing.drawRobot(packet.fieldOverlay(), start);
                Drawing.drawTags(packet.fieldOverlay(), start, initFrame);
            }
            dashboard.sendTelemetryPacket(packet);
        }
        if (isStopRequested()) {
            camera.stop();
            return;
        }
        if (!haveSettings) {
            // Refuse to run: no setPose, no hold, the motors stay off
            while (opModeIsActive()) {
                telemetry.addLine("NO GAME SETUP: autonomous not run. Run [John] Game Setup before the match.");
                telemetry.update();
            }
            camera.stop();
            return;
        }

        log = new CsvLog("john_auto", LOG_HEADER);
        clock.reset();
        Pose start = startPose(settings.startPose(), startFix);
        follower.setPose(start);
        watchdog.poseSet();
        follower.update();
        follower.hold(start);
        gamepad1.resetEdgeDetection();
        String startWarning = cameraWarning(initTimer.seconds(), initFrames, initFramesWithTags);
        if (startWarning != null) {
            logEvent("WARNING at START: " + startWarning);
        }
        logEvent(String.format(Locale.US, "start: %s %s at %s (%s), delay %.0f s", settings.alliance(), settings.tile(),
                Drawing.format(start), useCamera(startFix)
                        ? String.format(Locale.US, "camera Y, %d readings, tile %+.1f", startFix.count(),
                                startFix.y() - settings.startPose().y())
                        : "tile Y", settings.delayS()));

        Scheduler.reset();
        Scheduler.schedule(sequential(
                waitMs(settings.delayS() * 1000.0),
                race(mainRoutine(), waitUntil(() ->
                        AUTO_S - autoSeconds() <= parkSeconds(follower.pose()) + PARK_MARGIN_S)),
                instant(() -> {
                    if (!routineDone) {
                        logEvent(String.format(Locale.US, "time to park: %.1f s left, park about %.1f s from %s",
                                AUTO_S - autoSeconds(), parkSeconds(follower.pose()), Drawing.format(follower.pose())));
                    }
                    shooter.cancel();
                    shooter.setSpin(false);
                    intake.off();
                }),
                lazy(() -> routineDone ? label("park")
                        : instant(() -> step = "park")),   // forced by the time: no Y
                limit("park", PARK_MAX_S, lazy(this::park)),
                instant(() -> step = "done")));

        lastLoopS = clock.seconds();
        while (opModeIsActive()) {
            double nowS = clock.seconds();
            if (STEP_MODE && step.startsWith("press Y")) {
                waitedS += nowS - lastLoopS;   // waiting for Y is not autonomous time
            }
            lastLoopS = nowS;
            if (!Double.isNaN(intakeOffAtS) && autoSeconds() >= intakeOffAtS) {
                intakeOffAtS = Double.NaN;
                if (!intake.status().equals("off")) {
                    intake.off();
                    logEvent("intake off (after the GARDEN)");
                }
            }
            if (gamepad1.yWasPressed()) {
                advance = true;
            }
            follower.update();
            OdometryWatchdog.State state = watchdog.check(follower);
            frame = camera.poll();
            if (frame != null && !frame.tags.isEmpty()) {
                cameraSawTags = true;
            }
            tracker.update(frame, follower);
            if (tracker.event() != null) {
                logEvent(tracker.event());
            }
            Scheduler.execute();
            Pose pose = follower.pose();
            double[] target = tracker.target();
            shooter.setDistance(Math.hypot(target[0] - pose.x(), target[1] - pose.y()));
            String shooterEvent = shooter.update();
            if (shooterEvent != null) {
                logEvent(shooterEvent);
            }
            // Only good poses: at STOP the Pinpoint reads NOT_READY / zeros, and saving that handed the
            // TeleOp (0, 0, 0) (r0134 -> r0136)
            if (state == OdometryWatchdog.State.OK) {
                AutoSettings.savePose(blackboard, pose);
            }
            AutoSettings.saveUpAudience(blackboard, tracker.upAudience());

            telemetry.addData("Time", STEP_MODE ? "%.1f s autonomous (%.1f s with the Y waits)" : "%.1f s",
                    autoSeconds(), clock.seconds());
            telemetry.addData("Step", step);
            telemetry.addData("Alliance / tile", "%s / %s", settings.alliance(), settings.tile());
            telemetry.addData("Pose (FTC)", Drawing.format(pose));
            telemetry.addData("Target", "%s CELL (%.1f, %.1f), %s", tracker.upCellName(), target[0], target[1],
                    tracker.source());
            telemetry.addData("Shooter", shooter.status());
            telemetry.addData("Intake", intake.status());
            telemetry.addData("Camera", camera.status());
            telemetry.addData("Pinpoint", "%s, %s", state, watchdog.status());
            telemetry.addData("Blocked / pushed", "%d%s", watcher.trips(),
                    watcher.trips() > 0 ? ": last " + watcher.reason() : "");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            Drawing.drawTags(packet.fieldOverlay(), pose, frame);
            Drawing.drawTarget(packet.fieldOverlay(), null, target[0], target[1], "orange");
            dashboard.sendTelemetryPacket(packet);

            logRow(null);
        }
        logEvent("stop at " + Drawing.format(follower.pose()));
        shooter.stop();
        intake.off();
        camera.stop();
        log.close();
        watchdog.close();
    }

    // =========================================================================
    // Routine
    // =========================================================================

    private Command mainRoutine() {
        return sequential(
                label("step off the wall and aim 1"),
                limit("step off the wall and aim 1", STEP_AIM_MAX_S, sequential(lazy(this::stepOffWall), aim())),
                label("shoot 1"),
                limit("volley 1", SHOT_MAX_S, volley("volley 1", Shooter.INDEXER_STEPS_PER_VOLLEY, 0)),
                lazy(() -> emptied ? instant(() -> { }) : sequential(
                        instant(() -> logEvent("volley 1 did not finish: one more try")),
                        label("retry volley 1"),
                        guard("retry volley 1", () -> AIM_S + VOLLEY_S, follower::pose,
                                () -> limit("retry volley 1", VOLLEY_MAX_S + AIM_S,
                                        sequential(aim(), volley("volley 1 retry", Shooter.INDEXER_STEPS_PER_VOLLEY, 0)))))),
                label("garden"),
                lazy(() -> emptied
                        ? guard("garden", this::gardenSeconds, () -> field(gardenEnd()),
                                () -> limit("garden", GARDEN_MAX_S, lazy(this::collectGarden)))
                        : instant(() -> logEvent("GARDEN skipped: John may still hold balls (at most 4, G407)"))),
                label("look at the HIVE"),
                guard("look at the HIVE", () -> driveSeconds(follower.pose(), field(lookPose())) + LOOK_TIMEOUT_MS / 1000.0,
                        () -> field(lookPose()), () -> limit("look at the HIVE", LOOK_MAX_S, lazy(this::lookAtHive))),
                lazy(() -> {
                    if (!upKnown) {
                        return instant(() -> logEvent("no volley 2: the camera could not tell which CELL is up"));
                    }
                    return sequential(
                            label("to shot 2, aim 2, shoot 2"),
                            guard("shot 2", this::shotSeconds, this::shotEnd, () -> limit("shot 2", SHOT_MAX_S, sequential(
                                    lazy(this::toShootingSpot),
                                    aim(),
                                    volley("volley 2", Shooter.INDEXER_STEPS_PER_VOLLEY, 0)))));
                }),
                instant(() -> routineDone = true));
    }

    /**
     * Straight off the audience wall, just far enough to be at the aim heading without scraping, and
     * turning on the way (one path): at each point the heading is as far toward the aim as the
     * corner allows for the distance from the wall so far (safeHeading), so the turn is complete at
     * the end and the Aimer only has to refine.
     */
    private Command stepOffWall() {
        Pose start = red(follower.pose());
        double[] t = redTarget();
        double x = start.x();
        double aim = start.heading();
        for (int i = 0; i < 3; i++) {   // the aim heading depends a little on where we stop
            aim = ShootingZone.shootingHeading(x, start.y(), t[0], t[1]);
            x = AUDIENCE_WALL_X - maxHalfExtentX(start.heading(),
                    start.heading() + normalize(aim - start.heading())) - TURN_MARGIN;
        }
        double offX = Math.min(start.x(), x);
        double aimHeading = aim;
        logEvent(String.format(Locale.US, "step off the wall %.1f in, turning to %.1f deg on the way",
                start.x() - offX, Math.toDegrees(aimHeading)));
        return watcher.follow("step off the wall", () -> {
            // Re-planned from where the robot is (also on a retry)
            Pose from = red(follower.pose());
            Pose to = new Pose(offX, from.y(), aimHeading);
            return line(field(from), field(to)).heading((curve, s) -> {
                double redHeading = safeHeading(from, to, s);
                return blue ? normalize(redHeading + Math.PI) : redHeading;
            });
        });
    }

    /**
     * Red-frame heading at progress {@code s} (0..1) of the straight step from {@code from} to
     * {@code to}: the farthest turn from from's heading toward to's that keeps the robot's corner
     * TURN_MARGIN from the audience wall at that point (the whole sweep so far fits).
     */
    private static double safeHeading(Pose from, Pose to, double s) {
        s = Math.max(0.0, Math.min(1.0, s));
        double x = from.x() + (to.x() - from.x()) * s;
        double room = AUDIENCE_WALL_X - TURN_MARGIN - x;
        double turn = normalize(to.heading() - from.heading());
        double lo = 0.0, hi = 1.0;
        if (maxHalfExtentX(from.heading(), from.heading() + turn) <= room) {
            return to.heading();
        }
        for (int i = 0; i < 20; i++) {
            double mid = (lo + hi) / 2.0;
            if (maxHalfExtentX(from.heading(), from.heading() + turn * mid) <= room) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return normalize(from.heading() + turn * lo);
    }

    /** Aimer: turn to the up CELL, refine with the camera; the flywheel spins up meanwhile. */
    private Command aim() {
        return new CommandBuilder()
                .setStart(() -> {
                    shooter.setSpin(true);
                    aimResult = Aimer.Result.AIMING;
                    logEvent(aimer.start());
                })
                .setExecute(() -> {
                    if (aimResult == Aimer.Result.AIMING) {
                        aimResult = aimer.update();
                        if (aimer.event() != null) {
                            logEvent(aimer.event());
                        }
                    }
                })
                .setDone(() -> aimResult != Aimer.Result.AIMING);
    }

    /**
     * A volley of {@code steps} indexer steps at the up CELL, unless the aim failed. If the camera
     * sees the CELL start to tip, it stops, goes to the other side (whose CELL is now up), aims and
     * shoots the remaining steps (at most MAX_SIDE_SWITCHES times). The flywheel stops at the end.
     */
    private Command volley(String name, int steps, int switches) {
        final boolean[] tipped = {false};
        final boolean[] skipped = {false};
        final boolean[] wasAudience = {true};
        final double[] baseline = {Double.NaN};
        final int[] left = {0};
        Command fire = new CommandBuilder()
                .setStart(() -> {
                    tipped[0] = false;
                    skipped[0] = false;
                    emptied = false;
                    if (aimResult != Aimer.Result.AIMED) {
                        skipped[0] = true;
                        logEvent(name + " skipped: " + aimer.status());
                        return;
                    }
                    wasAudience[0] = tracker.upAudience();
                    baseline[0] = tracker.upHeight();
                    tracker.tipping(Double.NaN);   // clears the count
                    if (!intake.status().equals("off")) {
                        intake.off();
                        logEvent(name + ": intake off");
                    }
                    shooter.startVolley(steps);
                    logEvent(String.format(Locale.US, "%s: %d indexer steps at the %s CELL (tags at %.1f in)", name, steps,
                            tracker.upCellName(), baseline[0]));
                })
                .setExecute(() -> {
                    if (shooter.firing()) {
                        shooter.setPaused(!aimer.keepAim());   // bumped off aim: hold fire
                        if (aimer.event() != null) {
                            logEvent(name + ": " + aimer.event());
                        }
                    }
                    if (shooter.firing() && tracker.tipping(baseline[0])) {
                        left[0] = shooter.stepsLeft();
                        shooter.cancel();
                        tipped[0] = true;
                        logEvent(String.format(Locale.US, "%s: the %s CELL is tipping (tags at %.1f in, were %.1f): "
                                + "stop shooting, %d steps left", name, tracker.upCellName(), tracker.upHeight(),
                                baseline[0], left[0]));
                    }
                })
                .setDone(() -> !shooter.firing());
        return sequential(
                limit(name + ": firing", VOLLEY_MAX_S, fire),
                lazy(() -> {
                    shooter.setPaused(false);
                    if (!tipped[0]) {
                        emptied = !skipped[0] && !shooter.stuck() && shooter.stepsLeft() == 0;
                        return instant(() -> {
                            shooter.setSpin(false);
                            if (shooter.stuck()) {
                                logEvent("indexer stuck: the next volley tries again");
                            }
                        });
                    }
                    if (switches >= MAX_SIDE_SWITCHES) {
                        return instant(() -> {
                            shooter.setSpin(false);
                            logEvent(name + ": tipped again, no more side switches");
                        });
                    }
                    tracker.setUpAudience(!wasAudience[0]);
                    logEvent(name + ": going to the other side, the " + tracker.upCellName() + " CELL");
                    return sequential(
                            label(name + ": to the other side"),
                            guard(name + ": the other side", this::shotSeconds, this::shotEnd, () -> sequential(
                                    lazy(this::toShootingSpot),
                                    aim(),
                                    volley(name, Math.max(1, left[0]), switches + 1))));
                }));
    }

    /**
     * Drive beside our HIVE (LOOK_X, LOOK_Y, red frame) with the camera facing it, and wait for the
     * camera to confirm which CELL is up (upKnown). No assumption if it cannot tell.
     */
    private Command lookAtHive() {
        Pose look = lookPose();
        final ElapsedTime timer = new ElapsedTime();
        logEvent(String.format(Locale.US, "look at the HIVE from (%.1f, %.1f) red frame", LOOK_X, LOOK_Y));
        return sequential(
                watcher.follow("to look at the HIVE", () -> around(field(look), 0)),
                new CommandBuilder()
                        .setStart(() -> {
                            timer.reset();
                            upKnown = false;
                        })
                        .setDone(() -> {
                            double t = timer.milliseconds();
                            return (t >= LOOK_MIN_MS && tracker.msSinceConfirmed() < Math.min(t, CONFIRM_MAX_AGE_MS))
                                    || t >= LOOK_TIMEOUT_MS;
                        })
                        .setEnd(c -> {
                            double t = timer.milliseconds();
                            upKnown = tracker.msSinceConfirmed() < Math.min(t, CONFIRM_MAX_AGE_MS);
                            boolean seenAny = tracker.msSinceSeen() < t;
                            if (upKnown) {
                                logEvent("the camera confirms the " + tracker.upCellName() + " CELL is up");
                            } else if (!seenAny && NOT_SEEN_MEANS_TIPPED && camera.available() && cameraSawTags) {
                                // Red: the far CELL (field -X) is up; blue: the field audience CELL
                                tracker.setUpAudience(blue);
                                upKnown = true;
                                logEvent(String.format(Locale.US, "no tag of our HIVE in %.0f ms from the GARDEN side: it has "
                                        + "tipped, the %s CELL is up (NOT_SEEN_MEANS_TIPPED)", t, tracker.upCellName()));
                            } else {
                                logEvent(String.format(Locale.US, "the camera could not tell in %.0f ms%s", t,
                                        cameraSawTags ? "" : " (it has seen no HIVE tag in this run: not trusted to mean "
                                                + "tipped; check the exposure / gain)"));
                            }
                        }));
    }


    /**
     * Turn back to 90 deg in place, slide flush on the audience wall, then drive along it toward the
     * red corner with the intakes on: fast, then slow through the GARDEN balls.
     */
    private Command collectGarden() {
        Pose here = red(follower.pose());
        double halfLength = RobotGeometry.LENGTH_IN / 2.0;
        double wallX = AUDIENCE_WALL_X - RobotGeometry.WIDTH_IN / 2.0 - WALL_GAP;
        double heading = Math.toRadians(90.0);
        Pose turned = here.withHeading(heading);
        Pose atWall = new Pose(wallX, here.y(), heading);
        // Intake A is at the robot's -X end, which points toward the red wall (-Y) at heading 90
        double slowY = GARDEN_FIRST_BALL_Y + BALL_RADIUS + halfLength + SLOW_BEFORE_BALL_IN;
        double endY = RED_WALL_Y + halfLength + WALL_GAP;
        Pose slowStart = new Pose(wallX, Math.min(here.y(), slowY), heading);
        Pose end = new Pose(wallX, endY, heading);
        logEvent(String.format(Locale.US, "garden: wall at x %.1f, slow from y %.1f to %.1f at %.0f in/s",
                wallX, slowStart.y(), endY, COLLECT_SPEED_IPS));
        return sequential(
                instant(() -> follower.hold(field(turned))),
                race(waitUntil(() -> Math.abs(normalize(field(turned).heading() - follower.pose().heading()))
                                <= Math.toRadians(TURN_TOLERANCE_DEG)),
                        waitMs(TURN_TIMEOUT_MS)),
                watcher.follow("garden: to the wall", () -> around(field(atWall), 0)),
                intake.start(Intake.Side.A),
                watcher.follow("garden: along the wall", () -> toward(field(slowStart), true, 0)),
                watcher.creep("garden: collect", () -> field(end), COLLECT_SPEED_IPS),
                // The intake keeps running through the next stages, so the balls work their way to
                // the indexer while driving; the next volley (or the park) stops it
                instant(() -> {
                    intakeOffAtS = autoSeconds() + INTAKE_AFTER_GARDEN_S;
                    logEvent(String.format(Locale.US, "garden done: the intake keeps running %.1f s more", INTAKE_AFTER_GARDEN_S));
                }));
    }

    /**
     * To the nearest spot in the up CELL's shooting zone (ShootingZone, on our half, around the HIVE
     * frame), turning to the shooting heading on the way; nothing if already in range.
     */
    private Command toShootingSpot() {
        Pose here = follower.pose();
        double[] t = tracker.target();
        boolean audience = tracker.upAudience();
        String problem = ShootingZone.problem(here.x(), here.y(), t[0], t[1], audience);
        if (problem == null) {
            logEvent("shot 2: already in range of the " + tracker.upCellName() + " CELL");
            return instant(() -> { });
        }
        Pose spot = ShootingZone.nearestSpot(here, t[0], t[1], audience, !blue);
        if (spot == null) {
            logEvent("shot 2: " + problem + ", and no spot in range on the field");
            return instant(() -> { });
        }
        Pose goal = spot.withHeading(aimer.aimHeading(spot));
        logEvent(String.format(Locale.US, "shot 2 at the %s CELL: %s, driving to %s", tracker.upCellName(), problem,
                Drawing.format(goal)));
        return watcher.follow("to shot 2", () -> aimedRoute(goal));
    }

    /**
     * Around the HIVE frame to the LOADING ZONE; the last stretch slow. If that is blocked: stay when
     * already partly in the LOADING ZONE, else try the other spots along it.
     */
    private Command park() {
        double heading = Math.toRadians(PARK_HEADING_DEG);
        Pose approach = new Pose(PARK_APPROACH_X, parkY() + PARK_APPROACH_OFF_WALL_IN, heading);
        Pose parkPose = new Pose(PARK_X, parkY(), heading);
        logEvent(String.format(Locale.US, "park at (%.1f, %.1f) from %s", PARK_X, parkY(), Drawing.format(follower.pose())));
        return sequential(
                watcher.follow("park: around the HIVE", () -> around(field(approach), 0)),
                watcher.creep("park", () -> field(parkPose), PARK_SPEED_IPS),
                lazy(() -> {
                    if (!watcher.gaveUp()) {
                        return instant(() -> logEvent("parked at " + Drawing.format(follower.pose())));
                    }
                    List<Double> spots = new ArrayList<>();
                    double x = red(follower.pose()).x();
                    double a = PARK_X + PARK_ALT_OFFSET_IN;
                    double b = PARK_X - PARK_ALT_OFFSET_IN;
                    spots.add(Math.abs(a - x) <= Math.abs(b - x) ? a : b);
                    spots.add(Math.abs(a - x) <= Math.abs(b - x) ? b : a);
                    return parkElsewhere(spots);
                }));
    }

    /** After a blocked park: stay if partly in the LOADING ZONE, else try the next spot along it. */
    private Command parkElsewhere(List<Double> spots) {
        if (inLoadingZone(red(follower.pose()))) {
            return instant(() -> logEvent("park blocked, but already partly in the LOADING ZONE: staying at "
                    + Drawing.format(follower.pose())));
        }
        if (spots.isEmpty()) {
            return instant(() -> {
                follower.manual(DrivePowers.zero());
                logEvent("park: every spot blocked, NOT in the LOADING ZONE at " + Drawing.format(follower.pose()));
            });
        }
        double x = spots.remove(0);
        Pose spot = new Pose(x, parkY(), Math.toRadians(PARK_HEADING_DEG));
        return sequential(
                instant(() -> logEvent(String.format(Locale.US, "park blocked: trying x %.1f", x))),
                watcher.creep("park (other spot)", () -> field(spot), PARK_SPEED_IPS),
                lazy(() -> watcher.gaveUp() ? parkElsewhere(spots)
                        : instant(() -> logEvent("parked at " + Drawing.format(follower.pose())))));
    }

    /** The park line (red frame): the robot's half extent across the wall at the park heading, plus WALL_GAP. */
    private static double parkY() {
        double h = Math.toRadians(PARK_HEADING_DEG);
        double halfAcross = RobotGeometry.LENGTH_IN / 2.0 * Math.abs(Math.sin(h))
                + RobotGeometry.WIDTH_IN / 2.0 * Math.abs(Math.cos(h));
        return RED_WALL_Y + halfAcross + WALL_GAP;
    }

    /** True when the robot (red frame) overlaps the red LOADING ZONE by at least PARK_OVERLAP_IN both ways. */
    private static boolean inLoadingZone(Pose red) {
        double c = Math.abs(Math.cos(red.heading()));
        double s = Math.abs(Math.sin(red.heading()));
        double halfX = RobotGeometry.LENGTH_IN / 2.0 * c + RobotGeometry.WIDTH_IN / 2.0 * s;
        double halfY = RobotGeometry.LENGTH_IN / 2.0 * s + RobotGeometry.WIDTH_IN / 2.0 * c;
        double overlapX = Math.min(red.x() + halfX, LOADING_ZONE_MAX_X) - Math.max(red.x() - halfX, LOADING_ZONE_MIN_X);
        double overlapY = Math.min(red.y() + halfY, LOADING_ZONE_MAX_Y) - Math.max(red.y() - halfY, RED_WALL_Y);
        return overlapX >= PARK_OVERLAP_IN && overlapY >= PARK_OVERLAP_IN;
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    // =========================================================================
    // Time: estimates and stage guards
    // =========================================================================

    /**
     * Runs {@code stage} if its estimated time plus the park from where it ends ({@code end}, field
     * frame) still fits in the time left (keeping PARK_MARGIN_S); else skips it and everything
     * after it (parkNow). STEP_MODE: runs it anyway, the estimate is only logged.
     */
    private Command guard(String what, DoubleSupplier seconds, Supplier<Pose> end, Supplier<Command> stage) {
        return lazy(() -> {
            if (parkNow) {
                return instant(() -> logEvent(what + " skipped: going to park"));
            }
            double stageS = seconds.getAsDouble();
            double parkS = parkSeconds(end.get());
            double left = AUTO_S - autoSeconds();
            boolean fits = stageS + parkS + PARK_MARGIN_S <= left;
            String estimate = String.format(Locale.US, "%s: about %.1f s, then park %.1f s, %.1f s left", what, stageS,
                    parkS, left);
            if (!fits && !STEP_MODE) {
                parkNow = true;
                return instant(() -> logEvent(estimate + ": does not fit, going to park"));
            }
            logEvent(estimate + (fits ? "" : ": would not fit (step mode: run anyway)"));
            return stage.get();
        });
    }

    /** Autonomous time: since START, minus (in step mode) the time spent waiting for Y. */
    private double autoSeconds() {
        return clock.seconds() - (STEP_MODE ? waitedS : 0.0);
    }

    /** Estimated park time from a pose (field frame): around the HIVE to the approach, then slow. */
    private double parkSeconds(Pose from) {
        Pose approach = field(new Pose(PARK_APPROACH_X, parkY() + PARK_APPROACH_OFF_WALL_IN, Math.toRadians(PARK_HEADING_DEG)));
        Pose park = field(new Pose(PARK_X, parkY(), Math.toRadians(PARK_HEADING_DEG)));
        return routeLength(from, approach) / AVG_DRIVE_IPS
                + Math.hypot(park.x() - approach.x(), park.y() - approach.y()) / PARK_SPEED_IPS + PARK_OVERHEAD_S;
    }

    /** Estimated time to drive from a to b (field frame) around the HIVE. */
    private double driveSeconds(Pose a, Pose b) {
        return routeLength(a, b) / AVG_DRIVE_IPS + DRIVE_OVERHEAD_S;
    }

    private double routeLength(Pose a, Pose b) {
        List<double[]> route = ShootingZone.route(a, b, !blue);
        double length = 0;
        for (int i = 1; i < route.size(); i++) {
            length += Math.hypot(route.get(i)[0] - route.get(i - 1)[0], route.get(i)[1] - route.get(i - 1)[1]);
        }
        return length;
    }

    /** GARDEN estimate from here: to the wall, along it, slow through the balls. */
    private double gardenSeconds() {
        Pose here = follower.pose();
        double wallX = AUDIENCE_WALL_X - RobotGeometry.WIDTH_IN / 2.0 - WALL_GAP;
        Pose atWall = field(new Pose(wallX, red(here).y(), Math.toRadians(90.0)));
        double slowY = GARDEN_FIRST_BALL_Y + BALL_RADIUS + RobotGeometry.LENGTH_IN / 2.0 + SLOW_BEFORE_BALL_IN;
        double end = gardenEnd().y();
        double fast = Math.max(0.0, red(here).y() - slowY);
        double slowLength = Math.max(0.0, Math.min(red(here).y(), slowY) - end);
        return 0.5 + driveSeconds(here, atWall) + fast / AVG_DRIVE_IPS + slowLength / COLLECT_SPEED_IPS
                + Intake.SETTLE_MS / 1000.0;
    }

    /** Where the GARDEN run ends (red frame): in the red corner along the audience wall. */
    private static Pose gardenEnd() {
        return new Pose(AUDIENCE_WALL_X - RobotGeometry.WIDTH_IN / 2.0 - WALL_GAP,
                RED_WALL_Y + RobotGeometry.LENGTH_IN / 2.0 + WALL_GAP, Math.toRadians(90.0));
    }

    /** Where to look at the HIVE from (red frame), the shooter (and camera) facing its center. */
    private static Pose lookPose() {
        return new Pose(LOOK_X, LOOK_Y, ShootingZone.shootingHeading(LOOK_X, LOOK_Y, 0.0, -HiveTags.HIVE_Y));
    }

    /** Where the next shot is taken from (field frame): here if in range, else the nearest zone spot. */
    private Pose shotEnd() {
        Pose here = follower.pose();
        double[] t = tracker.target();
        if (ShootingZone.problem(here.x(), here.y(), t[0], t[1], tracker.upAudience()) == null) {
            return here;
        }
        Pose spot = ShootingZone.nearestSpot(here, t[0], t[1], tracker.upAudience(), !blue);
        return spot == null ? here : spot;
    }

    /** Shot estimate from here: drive to the spot (if needed), aim, volley. */
    private double shotSeconds() {
        Pose here = follower.pose();
        Pose spot = shotEnd();
        double drive = Math.hypot(spot.x() - here.x(), spot.y() - here.y()) < 0.5 ? 0.0 : driveSeconds(here, spot);
        return drive + AIM_S + VOLLEY_S;
    }

    // =========================================================================
    // Start position and other helpers
    // =========================================================================

    /**
     * A warning when, after CAMERA_CHECK_S of INIT, the camera is missing, sends no frames, or sees
     * no HIVE tag although John faces the HIVE from his start wall (then the Limelight's exposure /
     * gain is likely off, or the lens is covered); null when all is well.
     */
    private String cameraWarning(double initS, int frames, int framesWithTags) {
        if (!camera.available()) {
            return "NO CAMERA: " + camera.status();
        }
        if (initS < CAMERA_CHECK_S) {
            return null;
        }
        if (frames == 0) {
            return "THE LIMELIGHT SENDS NO FRAMES (pipeline " + HiveCamera.PIPELINE + " running? cable?)";
        }
        if (framesWithTags == 0) {
            return String.format(Locale.US, "THE CAMERA SEES NO HIVE TAGS in %d frames: check the Limelight "
                    + "exposure / gain (pipeline %d), the lens, and that John faces the HIVE", frames, HiveCamera.PIPELINE);
        }
        return null;
    }

    /** The camera's position along the wall is steady enough to use. */
    private static boolean useCamera(TagLocalizer fix) {
        return fix.count() >= MIN_START_SAMPLES && fix.ySpread() <= START_MAX_SPREAD_IN;
    }

    /** The Game Setup start pose, with Y from the camera when it is steady. */
    private static Pose startPose(Pose wall, TagLocalizer fix) {
        return useCamera(fix) ? fix.alongWall(wall, MIN_START_SAMPLES, RobotGeometry.LENGTH_IN / 2.0) : wall;
    }

    /** The target CELL in the red frame. */
    private double[] redTarget() {
        double[] t = tracker.target();
        return blue ? new double[]{-t[0], -t[1]} : t;
    }

    /**
     * A straight path from where the robot is now to {@code end} (field frame), keeping the end
     * heading (constant) or turning to it on the way; no faster than {@code speed} in/s if > 0.
     * Planned when the drive starts, and again from wherever the robot is on a PathWatcher retry.
     */
    private Path toward(Pose end, boolean constantHeading, double speed) {
        Pose here = follower.pose();
        Path p = constantHeading ? line(here, end).constant(end) : line(here, end).linear(here, end);
        return speed > 0 ? slow(p, speed) : p;
    }

    /**
     * From where the robot is now to {@code end} (field frame) around the HIVE frame
     * (ShootingZone.route), turning to the end heading on the first leg.
     */
    private Path around(Pose end, double speed) {
        Pose here = follower.pose();
        List<double[]> route = ShootingZone.route(here, end, !blue);
        Path[] legs = new Path[route.size() - 1];
        for (int i = 0; i < legs.length; i++) {
            Pose a = i == 0 ? here : new Pose(route.get(i)[0], route.get(i)[1], end.heading());
            Pose b = new Pose(route.get(i + 1)[0], route.get(i + 1)[1], end.heading());
            legs[i] = i == 0 ? line(a, b).linear(a, b) : line(a, b).constant(b);
        }
        Path p = legs.length == 1 ? legs[0] : path(legs);
        return speed > 0 ? slow(p, speed) : p;
    }

    /**
     * From where the robot is now to {@code end} (field frame) around the HIVE frame, the shooter
     * turning toward the target CELL along the way (Aimer.facingTarget), so it arrives aimed.
     */
    private Path aimedRoute(Pose end) {
        Pose here = follower.pose();
        List<double[]> route = ShootingZone.route(here, end, !blue);
        Path[] legs = new Path[route.size() - 1];
        for (int i = 0; i < legs.length; i++) {
            Pose a = i == 0 ? here : new Pose(route.get(i)[0], route.get(i)[1], end.heading());
            Pose b = new Pose(route.get(i + 1)[0], route.get(i + 1)[1], end.heading());
            legs[i] = line(a, b).heading(aimer.facingTarget(i == 0 ? here.heading() : null));
        }
        return legs.length == 1 ? legs[0] : path(legs);
    }

    /** A path driven no faster than {@code speed} in/s (Pedro reverts the limit after the path). */
    private Path slow(Path p, double speed) {
        return p.with(foresight.maxVelocityConstraint.at(speed));
    }

    /**
     * Largest half-extent of the robot along field X while turning between two headings: how far
     * its corner reaches toward the audience wall.
     */
    private static double maxHalfExtentX(double fromHeading, double toHeading) {
        double halfLength = RobotGeometry.LENGTH_IN / 2.0;
        double halfWidth = RobotGeometry.WIDTH_IN / 2.0;
        double worst = 0;
        for (int i = 0; i <= 50; i++) {
            double h = fromHeading + (toHeading - fromHeading) * i / 50.0;
            worst = Math.max(worst, halfLength * Math.abs(Math.cos(h)) + halfWidth * Math.abs(Math.sin(h)));
        }
        return worst;
    }

    /** Red-frame pose -> field pose for the current alliance (blue: reflected through the center). */
    private Pose field(Pose red) {
        return blue ? new Pose(-red.x(), -red.y(), normalize(red.heading() + Math.PI)) : red;
    }

    /** Field pose -> red-frame pose (the reflection is its own inverse). */
    private Pose red(Pose field) {
        return field(field);
    }

    /**
     * Runs {@code stage} for at most {@code seconds}; past that it is dropped: the shooter stops, the
     * robot holds where it is, and it is logged. No stage can hang the autonomous.
     */
    private Command limit(String name, double seconds, Command stage) {
        final boolean[] timedOut = {false};
        return sequential(
                instant(() -> timedOut[0] = false),
                race(stage, sequential(waitMs(seconds * 1000.0), instant(() -> timedOut[0] = true))),
                lazy(() -> !timedOut[0] ? instant(() -> { }) : instant(() -> {
                    shooter.cancel();
                    shooter.setPaused(false);
                    follower.hold(follower.pose());
                    logEvent(String.format(Locale.US, "%s: over its %.0f s limit, dropped at %s", name, seconds,
                            Drawing.format(follower.pose())));
                })));
    }

    /** Marks the start of a stage. In step mode it first waits for a press of gamepad1 Y. */
    private Command label(String name) {
        return sequential(
                instant(() -> {
                    advance = false;
                    if (STEP_MODE) {
                        step = "press Y: " + name;
                        logEvent("waiting for Y: " + name);
                    }
                }),
                waitUntil(() -> !STEP_MODE || consumeAdvance()),
                instant(() -> {
                    step = name;
                    logEvent("step " + name);
                }));
    }

    private boolean consumeAdvance() {
        if (advance) {
            advance = false;
            return true;
        }
        return false;
    }

    private void logRow(String event) {
        if (log == null) {
            return;
        }
        Pose p = follower.pose();
        double[] t = tracker.target();
        log.row(clock.seconds(), step, p.x(), p.y(), Math.toDegrees(p.heading()), follower.mode(),
                shooter.status(), intake.status(), t[0], t[1], tracker.source(), tracker.upCellName(),
                frame == null ? null : frame.ids(), watchdog.state(), event);
    }

    private void logEvent(String event) {
        logRow(event);
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
