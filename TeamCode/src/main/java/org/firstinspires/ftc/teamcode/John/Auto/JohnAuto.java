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
 *     with the intakes on: fast until just before the first ball, then COLLECT_SPEED_IPS.
 *  4. Look at the HIVE from beside it (LOOK_X, LOOK_Y, between its CELLS, camera facing it): its
 *     tags face the end that is up, so only from the side can they be read whether it tipped or
 *     not. Wait until the camera confirms which CELL is up (CellTracker); no assumption: if it
 *     cannot tell within LOOK_TIMEOUT_MS there is no second volley.
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
 * At PARK_DEADLINE_S whatever is still running is dropped and the robot goes to park. A failed aim
 * (more than Aimer.AIM_MAX_ERROR_DEG off) skips that volley.
 *
 * Other robots: every drive goes through PathWatcher. When John is BLOCKED (not moving with path
 * left) or PUSHED off the path, it stops pushing, waits, plans again from where it is and retries;
 * after PathWatcher.RETRIES it gives up and the routine goes on with the next stage. Parking (the
 * partner robot will likely park in the same LOADING ZONE): if the park is blocked, John stays if
 * he is already partly in the LOADING ZONE (that scores), otherwise he tries spots PARK_ALT_OFFSET_IN
 * to either side along it, the nearer one first.
 *
 * Slow stretches use Pedro's per-path modifier: path.with(config.maxVelocityConstraint.at(speed)).
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
    /** Park: partly inside the red LOADING ZONE (X -48 to -24, Y -72 to -60.4), heading 90. */
    public static double PARK_X = -36.0;
    public static double PARK_HEADING_DEG = 90.0;
    /** Other park spots along the LOADING ZONE, this far to either side of PARK_X, if the park is blocked. */
    public static double PARK_ALT_OFFSET_IN = 10.0;
    /** Red LOADING ZONE (FTC); PARK scores when the robot is partly inside (by at least PARK_OVERLAP_IN). */
    public static double LOADING_ZONE_MIN_X = -48.0;
    public static double LOADING_ZONE_MAX_X = -24.0;
    public static double LOADING_ZONE_MAX_Y = -60.4;
    public static double PARK_OVERLAP_IN = 1.0;
    /** Before the last park stretch; the drive there goes around the HIVE frame (ShootingZone.route). */
    public static double PARK_APPROACH_X = -24.0;
    public static double PARK_APPROACH_Y = -56.0;

    // ---- Start position from the camera ----
    public static int START_SAMPLES = 60;
    public static int MIN_START_SAMPLES = 5;
    /** Use the camera's Y only when its readings agree this well (median distance from the median). */
    public static double START_MAX_SPREAD_IN = 2.0;

    // ---- Looking at the HIVE after the GARDEN ----
    /**
     * Where to look from (red frame): beside our HIVE, between its CELLS, clear of the frame (Y -25,
     * plus the robot), camera facing the HIVE center.
     */
    public static double LOOK_X = 0.0;
    public static double LOOK_Y = -44.0;
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
    public static double PARK_SPEED_IPS = 15.0;
    /** Turning back to the wall before the GARDEN. */
    public static double TURN_TOLERANCE_DEG = 1.5;
    public static double TURN_TIMEOUT_MS = 1500;
    /** Go to park at this time after START no matter what (AUTO is 30 s; PARK counts at its end). */
    public static double PARK_DEADLINE_S = 25.0;
    /** Testing: wait for gamepad1 Y before each stage; the park deadline is off. */
    // public static boolean STEP_MODE = false;  // match
    public static boolean STEP_MODE = true;      // TEST: press Y to start each stage

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
    private HiveCamera.Frame frame;
    private final ElapsedTime clock = new ElapsedTime();
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
        while (opModeInInit()) {
            Pose wall = settings.startPose();
            startFix.add(camera.poll(), wall.heading());
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
        logEvent(String.format(Locale.US, "start: %s %s at %s (%s), delay %.0f s", settings.alliance(), settings.tile(),
                Drawing.format(start), useCamera(startFix)
                        ? String.format(Locale.US, "camera Y, %d readings, tile %+.1f", startFix.count(),
                                startFix.y() - settings.startPose().y())
                        : "tile Y", settings.delayS()));

        Scheduler.reset();
        Scheduler.schedule(sequential(
                waitMs(settings.delayS() * 1000.0),
                race(mainRoutine(), waitUntil(() -> !STEP_MODE && clock.seconds() >= PARK_DEADLINE_S)),
                instant(() -> {
                    shooter.cancel();
                    shooter.setSpin(false);
                    intake.off();
                }),
                label("park"),
                lazy(this::park),
                instant(() -> step = "done")));

        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                advance = true;
            }
            follower.update();
            OdometryWatchdog.State state = watchdog.check(follower);
            frame = camera.poll();
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
            AutoSettings.savePose(blackboard, pose);
            AutoSettings.saveUpAudience(blackboard, tracker.upAudience());

            telemetry.addData("Time", "%.1f s", clock.seconds());
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
                label("step off the wall and aim 1"), lazy(this::stepOffWall), aim(),
                label("shoot 1"), volley("volley 1", Shooter.INDEXER_STEPS_PER_VOLLEY, 0),
                label("garden"), lazy(this::collectGarden),
                label("look at the HIVE"), lazy(this::lookAtHive),
                lazy(() -> {
                    if (!upKnown) {
                        return instant(() -> logEvent("no volley 2: the camera could not tell which CELL is up"));
                    }
                    return sequential(
                            label("to shot 2"), lazy(this::toShootingSpot),
                            label("aim 2"), aim(),
                            label("shoot 2"), volley("volley 2", Shooter.INDEXER_STEPS_PER_VOLLEY, 0));
                }));
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
        final boolean[] wasAudience = {true};
        final double[] baseline = {Double.NaN};
        final int[] left = {0};
        Command fire = new CommandBuilder()
                .setStart(() -> {
                    tipped[0] = false;
                    if (aimResult != Aimer.Result.AIMED) {
                        logEvent(name + " skipped: " + aimer.status());
                        return;
                    }
                    wasAudience[0] = tracker.upAudience();
                    baseline[0] = tracker.upHeight();
                    tracker.tipping(Double.NaN);   // clears the count
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
                fire,
                lazy(() -> {
                    shooter.setPaused(false);
                    if (!tipped[0]) {
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
                            label(name + ": to the other side"), lazy(this::toShootingSpot),
                            aim(),
                            volley(name, Math.max(1, left[0]), switches + 1));
                }));
    }

    /**
     * Drive beside our HIVE (LOOK_X, LOOK_Y, red frame) with the camera facing it, and wait for the
     * camera to confirm which CELL is up (upKnown). No assumption if it cannot tell.
     */
    private Command lookAtHive() {
        // The camera looks out of the shooter side: aim the shooter at the HIVE center
        double heading = ShootingZone.shootingHeading(LOOK_X, LOOK_Y, 0.0, -HiveTags.HIVE_Y);
        Pose look = new Pose(LOOK_X, LOOK_Y, heading);
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
                            logEvent(upKnown ? "the camera confirms the " + tracker.upCellName() + " CELL is up"
                                    : String.format(Locale.US, "the camera did not see the HIVE in %.0f ms", t));
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
                watcher.follow("garden: collect", () -> toward(field(end), true, COLLECT_SPEED_IPS)),
                waitMs(Intake.SETTLE_MS),
                intake.stop());
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
        Pose approach = new Pose(PARK_APPROACH_X, PARK_APPROACH_Y, heading);
        Pose parkPose = new Pose(PARK_X, parkY(), heading);
        logEvent(String.format(Locale.US, "park at (%.1f, %.1f) from %s", PARK_X, parkY(), Drawing.format(follower.pose())));
        return sequential(
                watcher.follow("park: around the HIVE", () -> around(field(approach), 0)),
                watcher.follow("park", () -> around(field(parkPose), PARK_SPEED_IPS)),
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
                watcher.follow("park (other spot)", () -> around(field(spot), PARK_SPEED_IPS)),
                lazy(() -> watcher.gaveUp() ? parkElsewhere(spots)
                        : instant(() -> logEvent("parked at " + Drawing.format(follower.pose())))));
    }

    private static double parkY() {
        return RED_WALL_Y + RobotGeometry.LENGTH_IN / 2.0 + WALL_GAP;
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
