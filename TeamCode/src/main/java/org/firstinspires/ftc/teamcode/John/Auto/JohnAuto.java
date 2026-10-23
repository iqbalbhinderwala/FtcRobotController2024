package org.firstinspires.ftc.teamcode.John.Auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Hardware.Intake;
import org.firstinspires.ftc.teamcode.John.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.RobotGeometry;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

/**
 * John's autonomous. The alliance, start tile and start delay come from [John] Game Setup (run it
 * before the match; see AutoSettings). INIT only shows them and previews the start pose on the
 * Dashboard field; no buttons are used, so nothing can be changed while it waits. Without a saved
 * Game Setup it refuses to run: the robot does not move at all.
 *
 * Routine (all positions written for RED in FTC field coordinates, inches; BLUE is the same plan
 * reflected through the field center):
 *  1. Start (B1: ramp side flush on the audience wall, shooter facing the field). Step straight off
 *     the wall just far enough for the corner to clear the wall while turning.
 *  2. Aim: turn in place so the shooter points at the center of the up CELL (CELL_X, CELL_Y), and
 *     shoot the 4 preloaded balls.
 *  3. GARDEN: turn back, slide flush on the audience wall, then drive along it toward the red corner
 *     with intake A leading: fast until just before the first ball, then COLLECT_SPEED_IPS through
 *     the 4 balls to the corner.
 *  4. Go back to (SHOT2_X, SHOT2_Y), on the diagonal from the CELL toward the red corner, aim at the
 *     CELL and shoot.
 *  5. PARK in the LOADING ZONE, around the HIVE frame; the last stretch at PARK_SPEED_IPS.
 * At PARK_DEADLINE_S whatever is still running is dropped and the robot goes to park.
 *
 * Slow stretches use Pedro's per-path modifier: path.with(config.maxVelocityConstraint.at(speed)).
 * Shooter and intake are stubs for now (John/Hardware). STEP_MODE (testing) waits for gamepad1 Y
 * before each stage and turns the park deadline off.
 *
 * Every run is logged to /sdcard/FIRST/john_logs/r<run>_john_auto_<time>.csv: one row per loop
 * (step, pose, shooter, intake, watchdog) and an event row per decision. The final pose is kept in
 * the blackboard for a following TeleOp (AutoSettings.loadPose).
 */
@Config("JohnAuto")
@Autonomous(name = "[John] Auto", group = "John", preselectTeleOp = "[John] Drive Test")
public class JohnAuto extends LinearOpMode {

    // ---- Red positions, FTC field coordinates (inches); blue is reflected ----
    /** Center of the up CELL seen from above (red audience CELL, up at the match start). */
    public static double CELL_X = 13.0;
    public static double CELL_Y = -12.75;
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
    /** Second shot: on the 45 deg diagonal from the CELL toward the red audience corner (tile B2). */
    public static double SHOT2_X = 36.0;
    public static double SHOT2_Y = -36.0;
    /** Park: partly inside the red LOADING ZONE (X -48 to -24, Y -72 to -60.4), heading 90. */
    public static double PARK_X = -36.0;
    public static double PARK_HEADING_DEG = 90.0;
    /** Waypoints to the park, keeping well clear of the HIVE frame (X -20 to 20, Y -25 to 25). */
    public static double PARK_VIA_X = 14.0;
    public static double PARK_VIA_Y = -50.0;
    public static double PARK_APPROACH_X = -24.0;
    public static double PARK_APPROACH_Y = -56.0;

    // ---- Speeds and timing ----
    public static double COLLECT_SPEED_IPS = 12.0;
    public static double PARK_SPEED_IPS = 15.0;
    public static double AIM_TOLERANCE_DEG = 1.5;
    public static double AIM_TIMEOUT_MS = 1500;
    /** Go to park at this time after START no matter what (AUTO is 30 s; PARK counts at its end). */
    public static double PARK_DEADLINE_S = 25.0;
    /** Testing: wait for gamepad1 Y before each stage; the park deadline is off. */
    // public static boolean STEP_MODE = false;  // match
    public static boolean STEP_MODE = true;      // TEST: press Y to start each stage

    private static final String LOG_HEADER = "t_s,step,x,y,h_deg,follower,shooter,intake,watchdog,event";

    private Follower follower;
    private ForesightConfig foresight;
    private OdometryWatchdog watchdog;
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private boolean blue;
    private String step = "init";
    private boolean advance;
    private final ElapsedTime clock = new ElapsedTime();
    private CsvLog log;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        foresight = Constants.foresightConfig();
        follower = Constants.create(hardwareMap, foresight);
        watchdog = new OdometryWatchdog(hardwareMap, "auto");
        boolean haveSettings = AutoSettings.isSaved(blackboard);
        AutoSettings settings = new AutoSettings(blackboard);
        blue = settings.alliance() == AutoSettings.Alliance.BLUE;

        // INIT: show the Game Setup and the start pose (no buttons)
        while (opModeInInit()) {
            if (haveSettings) {
                telemetry.addLine("Game Setup:");
                settings.addTelemetry(telemetry, false);
                if (STEP_MODE) {
                    telemetry.addLine("STEP MODE (test): press Y to start each stage.");
                }
            } else {
                telemetry.addLine("NO GAME SETUP: this autonomous will not run.");
                telemetry.addLine("Stop, run [John] Game Setup, then INIT [John] Auto again.");
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            if (haveSettings) {
                Drawing.drawRobot(packet.fieldOverlay(), settings.startPose());
            }
            dashboard.sendTelemetryPacket(packet);
        }
        if (isStopRequested()) {
            return;
        }
        if (!haveSettings) {
            // Refuse to run: no setPose, no hold, the motors stay off
            while (opModeIsActive()) {
                telemetry.addLine("NO GAME SETUP: autonomous not run. Run [John] Game Setup before the match.");
                telemetry.update();
            }
            return;
        }

        log = new CsvLog("john_auto", LOG_HEADER);
        clock.reset();
        Pose start = settings.startPose();
        follower.setPose(start);
        watchdog.poseSet();
        follower.update();
        follower.hold(start);
        gamepad1.resetEdgeDetection();
        logEvent(String.format("start: %s %s at %s, delay %.0f s", settings.alliance(), settings.tile(),
                Drawing.format(start), settings.delayS()));

        Scheduler.reset();
        Scheduler.schedule(sequential(
                waitMs(settings.delayS() * 1000.0),
                race(mainRoutine(), waitUntil(() -> !STEP_MODE && clock.seconds() >= PARK_DEADLINE_S)),
                instant(() -> {
                    shooter.stop();
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
            Scheduler.execute();
            AutoSettings.savePose(blackboard, follower.pose());

            telemetry.addData("Time", "%.1f s", clock.seconds());
            telemetry.addData("Step", step);
            telemetry.addData("Alliance / tile", "%s / %s", settings.alliance(), settings.tile());
            telemetry.addData("Pose (FTC)", Drawing.format(follower.pose()));
            telemetry.addData("Shooter", shooter.status());
            telemetry.addData("Intake", intake.status());
            telemetry.addData("Pinpoint", "%s, %s", state, watchdog.status());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), follower.pose());
            dashboard.sendTelemetryPacket(packet);

            logRow(null);
        }
        logEvent("stop at " + Drawing.format(follower.pose()));
        log.close();
        watchdog.close();
    }

    // =========================================================================
    // Routine
    // =========================================================================

    private Command mainRoutine() {
        return sequential(
                label("step off the wall"), lazy(this::stepOffWall),
                label("aim 1"), lazy(this::aimAtCell),
                label("shoot 1"), shooter.shoot(4),
                label("garden"), lazy(this::collectGarden),
                label("to shot 2"), lazy(this::toSecondShot),
                label("aim 2"), lazy(this::aimAtCell),
                label("shoot 2"), shooter.shoot(4));
    }

    /** Straight off the audience wall, just far enough to turn to the aim heading without scraping. */
    private Command stepOffWall() {
        Pose start = red(follower.pose());
        double x = start.x();
        double aim = start.heading();
        for (int i = 0; i < 3; i++) {   // the aim heading depends a little on where we stop
            aim = aimHeading(x, start.y());
            x = AUDIENCE_WALL_X - maxHalfExtentX(aim, start.heading()) - TURN_MARGIN;
        }
        Pose off = new Pose(Math.min(start.x(), x), start.y(), start.heading());
        logEvent(String.format("step off the wall %.1f in to turn to %.1f deg", start.x() - off.x(),
                Math.toDegrees(aim)));
        return follow(follower, line(field(start), field(off)).constant(field(off)));
    }

    /** Turn in place so the shooter points at the CELL center; done when aligned or timed out. */
    private Command aimAtCell() {
        Pose robot = red(follower.pose());
        Pose aim = robot.withHeading(aimHeading(robot.x(), robot.y()));
        Pose target = field(aim);
        logEvent(String.format("aim at the CELL: heading %.1f deg (red frame) from (%.1f, %.1f)",
                Math.toDegrees(aim.heading()), robot.x(), robot.y()));
        return sequential(
                instant(() -> follower.hold(target)),
                race(waitUntil(() -> Math.abs(normalize(target.heading() - follower.pose().heading()))
                                <= Math.toRadians(AIM_TOLERANCE_DEG)),
                        waitMs(AIM_TIMEOUT_MS)));
    }

    /**
     * Turn back to 90 deg in place, slide flush on the audience wall, then drive along it toward the
     * red corner with intake A leading: fast, then slow through the GARDEN balls.
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
        logEvent(String.format("garden: wall at x %.1f, slow from y %.1f to %.1f at %.0f in/s",
                wallX, slowStart.y(), endY, COLLECT_SPEED_IPS));
        return sequential(
                instant(() -> follower.hold(field(turned))),
                race(waitUntil(() -> Math.abs(normalize(field(turned).heading() - follower.pose().heading()))
                                <= Math.toRadians(AIM_TOLERANCE_DEG)),
                        waitMs(AIM_TIMEOUT_MS)),
                follow(follower, line(field(turned), field(atWall)).constant(field(atWall))),
                intake.start(Intake.Side.A),
                follow(follower, line(field(atWall), field(slowStart)).constant(field(atWall))),
                follow(follower, slow(line(field(slowStart), field(end)).constant(field(end)), COLLECT_SPEED_IPS)),
                waitMs(Intake.SETTLE_MS),
                intake.stop());
    }

    /** Back to the second shooting spot on the diagonal, turning toward the aim heading on the way. */
    private Command toSecondShot() {
        Pose here = red(follower.pose());
        Pose spot = new Pose(SHOT2_X, SHOT2_Y, aimHeading(SHOT2_X, SHOT2_Y));
        return follow(follower, line(field(here), field(spot)).linear(field(here), field(spot)));
    }

    /** Around the HIVE frame to the LOADING ZONE; the last stretch slow. */
    private Command park() {
        Pose here = red(follower.pose());
        double heading = Math.toRadians(PARK_HEADING_DEG);
        double parkY = RED_WALL_Y + RobotGeometry.LENGTH_IN / 2.0 + WALL_GAP;
        Pose via = new Pose(PARK_VIA_X, PARK_VIA_Y, heading);
        Pose approach = new Pose(PARK_APPROACH_X, PARK_APPROACH_Y, heading);
        Pose parkPose = new Pose(PARK_X, parkY, heading);
        logEvent(String.format("park at (%.1f, %.1f) from %s", PARK_X, parkY, Drawing.format(follower.pose())));
        Path route = path(
                line(field(here), field(via)).linear(field(here), field(via)),
                line(field(via), field(approach)).constant(field(via)));
        return sequential(
                follow(follower, route),
                follow(follower, slow(line(field(approach), field(parkPose)).constant(field(parkPose)), PARK_SPEED_IPS)));
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /** A path driven no faster than {@code speed} in/s (Pedro reverts the limit after the path). */
    private Path slow(Path p, double speed) {
        return p.with(foresight.maxVelocityConstraint.at(speed));
    }

    /** Robot heading (red frame, radians) that points the shooter at the CELL center from (x, y). */
    private static double aimHeading(double x, double y) {
        return normalize(RobotGeometry.headingToShoot(Math.atan2(CELL_Y - y, CELL_X - x)));
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
        log.row(clock.seconds(), step, p.x(), p.y(), Math.toDegrees(p.heading()), follower.mode(),
                shooter.status(), intake.status(), watchdog.state(), event);
    }

    private void logEvent(String event) {
        logRow(event);
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
