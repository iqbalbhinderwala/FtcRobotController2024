package org.firstinspires.ftc.teamcode.John.TeleOp;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.algorithm.ForesightConfig;
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
import org.firstinspires.ftc.teamcode.John.Vision.TagLocalizer;

import java.util.List;
import java.util.Locale;

/**
 * John's TeleOp: field-centric driving, and one button that aims at the up CELL and shoots.
 *
 * START POSITION (chosen in INIT, dpad left / right):
 *  - AUTO: where [John] Auto ended (and which CELL it saw up), when it ended less than
 *    AUTO_POSE_MAX_AGE_S ago: the default right after the autonomous;
 *  - WALL: John's camera wall flush on the alliance's start wall (red: the audience wall; blue: the
 *    far wall), shooter facing the field, anywhere along the wall. The camera reads the HIVE tags; a
 *    tag's field Y never changes when the HIVE tips, so each tag gives the position along the wall
 *    (TagLocalizer, median over START_SAMPLES readings). X and heading come from the wall; the X
 *    the tags imply is shown as a check. Without a tag fix, the Game Setup tile (or C1) is used.
 * The alliance comes from [John] Game Setup if one was saved; B / X in INIT change it (red / blue).
 *
 * DRIVING: field-centric from the driver's view (red drivers face +Y, blue -Y): pushing the stick
 * away from you drives away from you, whatever way John faces. Pedro's ManualDrive.driveOrHold
 * holds the pose when the sticks are released (HOLD_WHEN_RELEASED).
 *
 * AIM AND SHOOT (A), the same parts as the autonomous:
 *  - Target: our HIVE's up CELL (CellTracker: which one from the tag heights, where from camera
 *    sightings while standing still, else its nominal place). Dpad down switches the CELL by hand
 *    (the camera wins when it sees tags).
 *  - Outside the shooting zone (ShootingZone: closer than 43.6", or more than 45 deg to the side of
 *    the way the CELL opening faces), A first drives to the nearest spot inside it, around the HIVE
 *    frame if needed, turning on the way (MOVE_SPEED_IPS). Blocked or pushed on the way
 *    (PathWatcher), it stops and leaves it to the driver.
 *  - On the way the shooter keeps turning toward the CELL (Aimer.facingTarget); then Aimer
 *    refines with the camera, and Shooter fires a volley at the speed for the distance, with the
 *    indexer's jam handling. Knocked off aim during the volley, it holds fire until back on aim;
 *    pushed away, it aims again from there (Aimer.keepAim).
 *  - Moving a stick, A again, or B cancels.
 *
 * gamepad1:
 *   left stick         drive (field-centric)
 *   right stick X      turn
 *   right bumper       slow (hold)
 *   A                  aim and shoot (again: cancel)
 *   Y                  flywheel pre-spin on/off (at the speed for the current distance)
 *   right / left trigger   intakes in / reverse (hold)
 *   dpad up            re-localize from the HIVE tags (stand still; uses the Pinpoint heading)
 *   dpad down          switch the target CELL (audience / far)
 *   B                  stop: cancel the shot, flywheel and pre-spin off
 *
 * OdometryWatchdog stops the drive on bad Pinpoint reads. Logged to
 * /sdcard/FIRST/john_logs/r<run>_john_teleop_<time>.csv.
 */
@Config("JohnTeleOp")
@TeleOp(name = "[John] TeleOp", group = "John")
public class JohnTeleOp extends LinearOpMode {

    // ---- Driving ----
    public static double DRIVE_SCALE = 1.0;
    public static double TURN_SCALE = 0.8;
    public static double SLOW_SCALE = 0.35;
    public static double STICK_DEADBAND = 0.05;
    public static boolean HOLD_WHEN_RELEASED = true;
    /** A stick moved this far cancels an aim / volley. */
    public static double CANCEL_STICK = 0.3;

    // ---- Start position and re-localizing ----
    /** Start from the autonomous' last pose when it is younger than this (at INIT). */
    public static double AUTO_POSE_MAX_AGE_S = 60.0;
    /** Tag readings kept for the start position (median). */
    public static int START_SAMPLES = 60;
    /** Fewer readings than this: no tag fix, use the Game Setup tile. */
    public static int MIN_START_SAMPLES = 5;
    /** Warn when the tags' X differs from the wall X by more than this (not flush, or mount / tag size off). */
    public static double X_CHECK_WARN_IN = 4.0;
    /** Re-localize: collect tags for this long while standing still. */
    public static double RELOCALIZE_MS = 500;
    public static int MIN_RELOCALIZE_SAMPLES = 4;

    // ---- Driving into the shooting zone ----
    public static double MOVE_SPEED_IPS = 40.0;
    public static double MOVE_TIMEOUT_MS = 6000;

    private enum Shot { IDLE, MOVE, AIM, FIRE }

    private enum Start { AUTO, WALL }

    private static final String LOG_HEADER = "t_s,shot,x,y,h_deg,aim_deg,target_x,target_y,target_src,"
            + "distance,rpm_target,rpm,flywheel_mode,indexer_step,tags,up_cell,watchdog,event";

    private Follower follower;
    private ForesightConfig foresight;
    private PathWatcher watcher;
    private OdometryWatchdog watchdog;
    private HiveCamera camera;
    private CellTracker tracker;
    private Aimer aimer;
    private Shooter shooter;
    private boolean red = true;

    // Shot
    private Shot shot = Shot.IDLE;
    private Pose moveGoal;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private String shotStatus = "";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        foresight = Constants.foresightConfig();
        follower = Constants.create(hardwareMap, foresight);
        watcher = new PathWatcher(follower, e -> { });
        watchdog = new OdometryWatchdog(hardwareMap, "teleop");
        camera = new HiveCamera(hardwareMap);
        shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        AutoSettings settings = new AutoSettings(blackboard);
        boolean haveSetup = AutoSettings.isSaved(blackboard);
        red = settings.alliance() == AutoSettings.Alliance.RED;
        TagLocalizer startFix = new TagLocalizer(START_SAMPLES);
        CsvLog log = new CsvLog("john_teleop", LOG_HEADER);
        Pose autoPose = AutoSettings.loadPose(blackboard);
        double autoAgeS = AutoSettings.poseAgeS(blackboard);
        Boolean autoUpAudience = AutoSettings.loadUpAudience(blackboard);
        Start startFrom = autoPose != null && autoAgeS <= AUTO_POSE_MAX_AGE_S ? Start.AUTO : Start.WALL;

        // ---- INIT: nothing moves; the camera finds the position along the wall ----
        gamepad1.resetEdgeDetection();
        while (opModeInInit()) {
            if (gamepad1.bWasPressed() && !red) {
                red = true;
                startFix.reset();
            }
            if (gamepad1.xWasPressed() && red) {
                red = false;
                startFix.reset();
            }
            if ((gamepad1.dpadLeftWasPressed() | gamepad1.dpadRightWasPressed()) && autoPose != null) {
                startFrom = startFrom == Start.AUTO ? Start.WALL : Start.AUTO;
            }
            Pose wall = wallPose(settings);
            startFix.add(camera.poll(), wall.heading());
            Pose start = startFrom == Start.AUTO ? autoPose : startFix.alongWall(wall, MIN_START_SAMPLES, RobotGeometry.LENGTH_IN / 2.0);

            telemetry.addData("Alliance", "%s   (B red, X blue)%s", red ? "RED" : "BLUE",
                    haveSetup ? "" : "   (no Game Setup saved)");
            telemetry.addData("Start from", "%s   (dpad left / right)%s", startFrom,
                    autoPose == null ? ": no autonomous pose" : String.format(Locale.US, ": the autonomous ended %.0f s ago%s",
                            autoAgeS, autoAgeS > AUTO_POSE_MAX_AGE_S ? " (old)" : ""));
            if (startFrom == Start.WALL) {
                telemetry.addLine(String.format("Place John with the camera wall flush on the %s wall, shooter facing the field.",
                        red ? "audience" : "far"));
                if (startFix.count() >= MIN_START_SAMPLES) {
                    telemetry.addData("Along the wall (tags)", "Y %.1f in (spread %.1f, %d readings)",
                            start.y(), startFix.ySpread(), startFix.count());
                    double xCheck = startFix.x() - wall.x();
                    telemetry.addData("X check", "tags say %.1f, wall %.1f (%+.1f)%s", startFix.x(), wall.x(), xCheck,
                            Math.abs(xCheck) > X_CHECK_WARN_IN ? "  <- NOT FLUSH, or camera mount / tag size off?" : "");
                } else {
                    telemetry.addData("Along the wall", "NO TAG FIX (%d readings): would use Y %.1f (%s)",
                            startFix.count(), wall.y(), haveSetup ? "Game Setup tile " + settings.tile() : "tile C1");
                }
            }
            telemetry.addData("Start pose (FTC)", Drawing.format(start));
            telemetry.addData("Camera", camera.status());
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), start);
            dashboard.sendTelemetryPacket(packet);
        }
        if (isStopRequested()) {
            camera.stop();
            log.close();
            watchdog.close();
            return;
        }

        Pose start = startFrom == Start.AUTO ? autoPose
                : startFix.alongWall(wallPose(settings), MIN_START_SAMPLES, RobotGeometry.LENGTH_IN / 2.0);
        follower.setPose(start);
        watchdog.poseSet();
        follower.update();
        tracker = new CellTracker(red);   // match start: red audience / blue far CELL up
        if (startFrom == Start.AUTO && autoUpAudience != null) {
            tracker.setUpAudience(autoUpAudience);
        }
        aimer = new Aimer(follower, tracker, camera.available());
        gamepad1.resetEdgeDetection();
        ElapsedTime clock = new ElapsedTime();
        boolean prespin = false;
        boolean holdMode = HOLD_WHEN_RELEASED;
        TagLocalizer relocalizer = new TagLocalizer(100);
        boolean relocalizing = false;
        ElapsedTime relocalizeTimer = new ElapsedTime();
        log.row(0.0, shot, start.x(), start.y(), Math.toDegrees(start.heading()), null, null, null, null, null, null,
                null, null, null, null, tracker.upCellName(), watchdog.state(),
                String.format(Locale.US, "start %s at %s (%s)", red ? "red" : "blue", Drawing.format(start),
                        startFrom == Start.AUTO ? "from the autonomous"
                                : startFix.count() >= MIN_START_SAMPLES ? startFix.count() + " tag readings" : "no tag fix"));

        while (opModeIsActive()) {
            String event = null;
            boolean shootPressed = gamepad1.aWasPressed(), spinPressed = gamepad1.yWasPressed();
            boolean stopPressed = gamepad1.bWasPressed();
            boolean relocalizePressed = gamepad1.dpadUpWasPressed(), switchPressed = gamepad1.dpadDownWasPressed();

            double stickFwd = deadband(-gamepad1.left_stick_y);
            double stickLeft = deadband(-gamepad1.left_stick_x);
            double stickTurn = deadband(-gamepad1.right_stick_x);
            boolean sticksMoved = Math.max(Math.abs(stickFwd), Math.max(Math.abs(stickLeft), Math.abs(stickTurn)))
                    > CANCEL_STICK;

            // ---- Camera: the tracker uses only frames taken standing still ----
            Pose pose = follower.pose();
            HiveCamera.Frame frame = camera.poll();
            if (tracker.update(frame, follower) && relocalizing) {
                relocalizer.add(frame, pose.heading());
            }
            if (tracker.event() != null) {
                event = tracker.event();
            }

            // ---- Buttons ----
            if (stopPressed) {
                cancelShot("stop");
                prespin = false;
                relocalizing = false;
                event = "stop";
            }
            if (spinPressed) {
                prespin = !prespin;
                event = prespin ? "pre-spin on" : "pre-spin off";
            }
            if (switchPressed) {
                tracker.setUpAudience(!tracker.upAudience());
                event = "target switched to the " + tracker.upCellName() + " CELL";
            }
            if (relocalizePressed && shot == Shot.IDLE && !relocalizing) {
                relocalizing = true;
                relocalizer.reset();
                relocalizeTimer.reset();
                event = "re-localize: stand still";
            }
            if (shootPressed) {
                if (shot != Shot.IDLE) {
                    cancelShot("cancelled (A)");
                    event = "shot cancelled";
                } else if (!watchdog.ok()) {
                    shotStatus = "can't aim: odometry " + watchdog.state();
                    event = shotStatus;
                } else {
                    event = requestShot();
                }
            }
            if (shot != Shot.IDLE && sticksMoved) {
                cancelShot("cancelled: driver took over");
                event = "shot cancelled: sticks";
            }
            if (shot != Shot.IDLE && watchdog.lost()) {
                cancelShot("cancelled: odometry lost");
                event = "shot cancelled: odometry lost";
            }

            // ---- Re-localize ----
            if (relocalizing) {
                if (sticksMoved) {
                    relocalizing = false;
                    event = "re-localize cancelled: moved";
                } else if (relocalizeTimer.milliseconds() >= RELOCALIZE_MS) {
                    relocalizing = false;
                    if (relocalizer.count() >= MIN_RELOCALIZE_SAMPLES) {
                        Pose before = follower.pose();
                        Pose fixed = new Pose(relocalizer.x(), relocalizer.y(), before.heading());
                        follower.setPose(fixed);
                        watchdog.poseSet();
                        follower.manual(DrivePowers.zero());   // drop a hold in the old coordinates
                        event = String.format(Locale.US, "re-localized from %d tag readings: %s -> %s",
                                relocalizer.count(), Drawing.format(before), Drawing.format(fixed));
                    } else {
                        event = "re-localize failed: " + relocalizer.count() + " tag readings (HIVE in view? standing still?)";
                    }
                }
            }

            // ---- Aim and shoot ----
            String shotEvent = updateShot();
            if (shotEvent != null) {
                event = shotEvent;
            }

            // ---- Drive (while moving / aiming / shooting the follower drives itself) ----
            boolean slow = gamepad1.right_bumper;
            if (shot == Shot.IDLE) {
                double scale = slow ? SLOW_SCALE : 1.0;
                DrivePowers powers = ManualDrive.fieldCentric(stickFwd * DRIVE_SCALE * scale,
                        stickLeft * DRIVE_SCALE * scale, stickTurn * TURN_SCALE * scale,
                        pose.heading(), -driverFacing());   // Pedro turns the sticks by -(heading + offset)
                if (holdMode) {
                    ManualDrive.driveOrHold(follower, powers);
                } else {
                    follower.manual(powers);
                }
            }
            follower.update();
            OdometryWatchdog.State state = watchdog.check(follower);
            pose = follower.pose();

            // ---- Shooter, intakes ----
            double[] target = tracker.target();
            double distance = Math.hypot(target[0] - pose.x(), target[1] - pose.y());
            shooter.setDistance(distance);
            shooter.setSpin(shot != Shot.IDLE || prespin);
            String shooterEvent = shooter.update();
            if (shooterEvent != null) {
                event = event == null ? shooterEvent : event + "; " + shooterEvent;
            }
            intake.set(gamepad1.right_trigger > 0.2 ? Intake.POWER : gamepad1.left_trigger > 0.2 ? -Intake.POWER : 0.0);

            // ---- Telemetry, Dashboard, log ----
            if (watchdog.lost()) {
                telemetry.addLine("ODOMETRY LOST, drive stopped: " + watchdog.reason());
            } else if (!watchdog.ok()) {
                telemetry.addLine("ODOMETRY PAUSED: " + watchdog.reason());
            }
            telemetry.addData("Shot (A)", "%s%s", shot, shotStatus.isEmpty() ? "" : ": " + shotStatus);
            telemetry.addData("Target", "%s %s CELL at (%.1f, %.1f), %.1f in away (%s)", red ? "red" : "blue",
                    tracker.upCellName(), target[0], target[1], distance, tracker.source());
            telemetry.addData("Shooter (Y pre-spin)", shooter.status());
            String zone = ShootingZone.problem(pose.x(), pose.y(), target[0], target[1], tracker.upAudience());
            telemetry.addData("Shooting zone", zone == null
                    ? String.format(Locale.US, "IN RANGE (%.0f deg off the CELL axis)",
                            ShootingZone.offAxisDeg(pose.x(), pose.y(), target[0], target[1], tracker.upAudience()))
                    : zone + ": A drives into range first");
            telemetry.addData("Pose (FTC)", Drawing.format(pose));
            if (relocalizing) {
                telemetry.addData("Re-localizing", "%d tag readings", relocalizer.count());
            }
            telemetry.addData("Camera", camera.status());
            telemetry.addData("Pinpoint", "%s, %s", state, watchdog.status());
            telemetry.addLine("A aim+shoot, Y pre-spin, B stop, RT/LT intakes, RB slow, dpad up re-localize, dpad down switch CELL");
            telemetry.update();

            Pose aimPose = shot == Shot.AIM || shot == Shot.FIRE ? aimer.aimPose() : null;
            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            Drawing.drawTarget(packet.fieldOverlay(), shot != Shot.IDLE ? pose : null, target[0], target[1], "orange");
            if (aimPose != null) {
                Drawing.drawRobotOutline(packet.fieldOverlay(), aimPose, "orange", null);
            }
            if (moveGoal != null && shot == Shot.MOVE) {
                Drawing.drawRobotOutline(packet.fieldOverlay(), moveGoal, "orange", "shoot here");
            }
            dashboard.sendTelemetryPacket(packet);

            log.row(clock.seconds(), shot, pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                    aimPose == null ? null : Math.toDegrees(aimPose.heading()), target[0], target[1], tracker.source(),
                    distance, shooter.flywheel().isOn() ? shooter.flywheel().targetRpm() : 0.0, shooter.flywheel().rpm(),
                    shooter.flywheel().mode(), shooter.indexer().stepCount(), frame == null ? null : frame.ids(),
                    tracker.upCellName(), state, event);
        }
        shooter.stop();
        intake.off();
        camera.stop();
        log.close();
        watchdog.close();
    }

    // =========================================================================
    // Start position
    // =========================================================================

    /** Start pose for the alliance with the Game Setup tile (or C1): X and heading are the wall's. */
    private Pose wallPose(AutoSettings settings) {
        return AutoSettings.startPose(red ? AutoSettings.Alliance.RED : AutoSettings.Alliance.BLUE, settings.tile());
    }

    // =========================================================================
    // Aim and shoot
    // =========================================================================

    /** A: aim and shoot from here if in the shooting zone, else drive into it first. */
    private String requestShot() {
        Pose here = follower.pose();
        double[] t = tracker.target();
        String problem = ShootingZone.problem(here.x(), here.y(), t[0], t[1], tracker.upAudience());
        if (problem == null) {
            return startAim();
        }
        Pose spot = ShootingZone.nearestSpot(here, t[0], t[1], tracker.upAudience(), red);
        if (spot == null) {
            shotStatus = problem + ", and no spot in range on the field";
            return "no shot: " + shotStatus;
        }
        spot = spot.withHeading(aimer.aimHeading(spot));   // with the trim
        List<double[]> route = ShootingZone.route(here, spot, red);
        follower.follow(routePath(here, route, spot));
        watcher.start();
        moveGoal = spot;
        shot = Shot.MOVE;
        moveTimer.reset();
        double length = 0;
        for (int i = 1; i < route.size(); i++) {
            length += Math.hypot(route.get(i)[0] - route.get(i - 1)[0], route.get(i)[1] - route.get(i - 1)[1]);
        }
        shotStatus = String.format(Locale.US, "%s: driving %.0f in to %s%s", problem, length, Drawing.format(spot),
                route.size() > 2 ? " around the HIVE" : "");
        return "move to shoot: " + shotStatus;
    }

    /**
     * The route as one path at MOVE_SPEED_IPS, the shooter turning toward the CELL along the way
     * (Aimer.facingTarget, blending from the start heading), so it arrives aimed.
     */
    private Path routePath(Pose start, List<double[]> route, Pose goal) {
        Path[] legs = new Path[route.size() - 1];
        for (int i = 0; i < legs.length; i++) {
            Pose a = new Pose(route.get(i)[0], route.get(i)[1], start.heading());
            Pose b = new Pose(route.get(i + 1)[0], route.get(i + 1)[1], goal.heading());
            legs[i] = line(a, b).heading(aimer.facingTarget(i == 0 ? start.heading() : null));
        }
        Path p = legs.length == 1 ? legs[0] : path(legs);
        return p.with(foresight.maxVelocityConstraint.at(MOVE_SPEED_IPS));
    }

    private String startAim() {
        shot = Shot.AIM;
        shotStatus = "turning";
        return aimer.start();
    }

    private void cancelShot(String why) {
        if (shot != Shot.IDLE) {
            follower.manual(DrivePowers.zero());
        }
        shooter.cancel();
        shot = Shot.IDLE;
        moveGoal = null;
        shotStatus = why;
    }

    /** Runs the move / aim / volley; returns an event or null. */
    private String updateShot() {
        Pose pose = follower.pose();
        switch (shot) {
            case MOVE:
                if (moveTimer.milliseconds() > 100 && !follower.following()) {
                    Pose goal = moveGoal;
                    moveGoal = null;
                    return String.format(Locale.US, "at the shooting spot (%.1f in from %s); %s",
                            Math.hypot(pose.x() - goal.x(), pose.y() - goal.y()), Drawing.format(goal), startAim());
                }
                if (watcher.check() != PathWatcher.Trip.NONE) {
                    cancelShot("stopped: " + watcher.reason());
                    return shotStatus;
                }
                if (moveTimer.milliseconds() > MOVE_TIMEOUT_MS) {
                    cancelShot("could not reach the shooting spot");
                    return shotStatus;
                }
                return null;
            case AIM: {
                Aimer.Result r = aimer.update();
                shotStatus = aimer.status();
                if (r == Aimer.Result.AIMED) {
                    shot = Shot.FIRE;
                    shooter.startVolley();
                    shotStatus = "firing";
                } else if (r == Aimer.Result.FAILED) {
                    cancelShot(aimer.status());
                }
                return aimer.event();
            }
            case FIRE:
                if (shooter.firing()) {
                    shooter.setPaused(!aimer.keepAim());   // bumped off aim: hold fire
                }
                if (!shooter.firing()) {
                    shooter.setPaused(false);
                    String done = shooter.stuck() ? "indexer stuck: clear the ball, then A again" : "volley done";
                    cancelShot(done);
                    return done;
                }
                shotStatus = shooter.status();
                return aimer.event();
            default:
                return null;
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /** Field direction the driver looks along (FTC, radians): red +90 deg, blue -90 deg. */
    private double driverFacing() {
        return Math.toRadians(red ? 90.0 : -90.0);
    }

    private static double deadband(double v) {
        return Math.abs(v) < STICK_DEADBAND ? 0.0 : v;
    }
}
