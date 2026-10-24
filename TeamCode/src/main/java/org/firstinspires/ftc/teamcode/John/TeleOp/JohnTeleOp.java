package org.firstinspires.ftc.teamcode.John.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Auto.AutoSettings;
import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Hardware.Flywheel;
import org.firstinspires.ftc.teamcode.John.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.John.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.RobotGeometry;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;
import org.firstinspires.ftc.teamcode.John.Vision.HiveCamera;
import org.firstinspires.ftc.teamcode.John.Vision.HiveTags;
import org.firstinspires.ftc.teamcode.John.Vision.TagLocalizer;

import java.util.ArrayDeque;
import java.util.Locale;

/**
 * John's TeleOp: field-centric driving, starting from a wall position found with the HIVE tags,
 * and one button that aims at the up CELL and shoots.
 *
 * START POSITION: like the autonomous, John's camera wall flush on the alliance's start wall (red:
 * the audience wall; blue: the far wall), shooter facing the field, anywhere along the wall. During
 * INIT the camera (looking out of the shooter side, toward the HIVE) reads the HIVE tags; a tag's
 * field Y never changes when the HIVE tips, so each tag gives the robot's position along the wall
 * (TagLocalizer, median over START_SAMPLES tag readings). X and heading come from the wall. The X the
 * tags imply is shown as a check. Without a tag fix, the Game Setup start tile (or C1) is used.
 * The alliance comes from [John] Game Setup if one was saved; B / X in INIT change it (red / blue).
 *
 * DRIVING: field-centric from the driver's view (red drivers face +Y, blue -Y): pushing the stick
 * away from you drives away from you, whatever way John faces. Pedro's ManualDrive.driveOrHold
 * holds the pose when the sticks are released (HOLD_WHEN_RELEASED).
 *
 * AIM AND SHOOT (A): John turns in place so the shooter points at the up CELL of our HIVE, then
 * fires a volley (Shooter.INDEXER_STEPS_PER_VOLLEY indexer steps) once the flywheel is at speed.
 *  - Target: the up CELL's tag cluster center, as seen by the camera whenever John stands still
 *    (from the same odometry pose used to aim, so a small odometry drift cancels out); without a
 *    recent sighting, its nominal position (HiveTags). The camera looks out of the shooter side, so
 *    once roughly aimed it sees the CELL and the aim is refined (up to MAX_REFINES times).
 *  - Which CELL is up comes from the tag heights (up ~50", down ~36"); at the start it is the red
 *    audience / blue far CELL. Dpad down switches it by hand (the camera wins when it sees tags).
 *  - Flywheel speed from the distance to the CELL (RPM_DISTANCES_IN / RPM_VALUES, from the
 *    2026-10-23 tests at C1 / B1 / A1); clamped outside that range.
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
    /** Tag readings kept for the start position (median). */
    public static int START_SAMPLES = 60;
    /** Fewer readings than this: no tag fix, use the Game Setup tile. */
    public static int MIN_START_SAMPLES = 5;
    /** Warn when the tags' X differs from the wall X by more than this (not flush, or mount / tag size off). */
    public static double X_CHECK_WARN_IN = 4.0;
    /** Re-localize: collect tags for this long while standing still. */
    public static double RELOCALIZE_MS = 500;
    public static int MIN_RELOCALIZE_SAMPLES = 4;
    /** "Standing still" for the camera: slower than this. */
    public static double STILL_SPEED_IPS = 2.0;
    public static double STILL_TURN_DPS = 10.0;

    // ---- Aiming ----
    public static double AIM_TOLERANCE_DEG = 1.5;
    /** Aligned and still for this long before refining / firing. */
    public static double AIM_SETTLE_MS = 150;
    /** After this long, fire if within AIM_MAX_ERROR_DEG, else give up. */
    public static double AIM_TIMEOUT_MS = 2500;
    public static double AIM_MAX_ERROR_DEG = 4.0;
    /** Re-aims with a newer camera sighting of the CELL. */
    public static int MAX_REFINES = 3;
    /** Added to the aim heading (counter-clockwise), if the shots pull to one side. */
    public static double AIM_TRIM_DEG = 0.0;
    /** Camera sightings of the CELL older than this are not used. */
    public static double VISION_MAX_AGE_MS = 1500;
    /** Aimed at the nominal CELL: wait this long for the camera to see it before firing anyway. */
    public static double VISION_WAIT_MS = 400;
    /** Frames in a row that must agree before the up CELL changes. */
    public static int UP_CELL_FRAMES = 3;

    // ---- Shooting ----
    /**
     * Flywheel RPM by horizontal distance from the robot center to the CELL center (inches): the
     * 2026-10-23 tests at C1 / B1 / A1 (camera wall on the audience wall, Shooter.RPM_FROM_*).
     */
    public static String RPM_DISTANCES_IN = "50.3,55.5,69.0";
    public static String RPM_VALUES = "2200,2300,2400";
    /** Flywheel without its encoder (open-loop): wait this long after switching it on. */
    public static double OPEN_LOOP_SPIN_UP_MS = 1500;
    public static double STEP_TIMEOUT_MS = 1000;

    // ---- Intakes (as in the Mechanism Test) ----
    public static double INTAKE_POWER = 1.0;
    public static boolean INTAKE_A_REVERSED = false;
    public static boolean INTAKE_B_REVERSED = false;

    private enum Shot { IDLE, AIM, FIRE }

    private enum Feed { SPIN_UP, STEP, INTERVAL }

    private static final String LOG_HEADER = "t_s,shot,x,y,h_deg,aim_deg,target_x,target_y,target_src,"
            + "distance,rpm_target,rpm,flywheel_mode,indexer_step,tags,up_cell,watchdog,event";

    private Follower follower;
    private OdometryWatchdog watchdog;
    private HiveCamera camera;
    private Flywheel flywheel;
    private Indexer indexer;
    private boolean red = true;

    // Target CELL: our HIVE's up CELL (audience or far) and recent camera sightings of it
    private boolean targetAudience;
    private int disagreeFrames;
    private final ArrayDeque<double[]> sightings = new ArrayDeque<>();   // {time ms, x, y}
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime stillFor = new ElapsedTime();
    private boolean still;

    // Shot
    private Shot shot = Shot.IDLE;
    private Feed feed = Feed.SPIN_UP;
    private Pose aimPose;
    private int refines;
    private int stepsDone;
    private final ElapsedTime aimTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private double alignedSinceMs = -1;
    private String shotStatus = "";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = Constants.create(hardwareMap);
        watchdog = new OdometryWatchdog(hardwareMap, "teleop");
        camera = new HiveCamera(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        indexer = new Indexer(hardwareMap);
        DcMotorEx intakeA = hardwareMap.get(DcMotorEx.class, "intakeA");
        DcMotorEx intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        for (DcMotorEx intake : new DcMotorEx[]{intakeA, intakeB}) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        AutoSettings settings = new AutoSettings(blackboard);
        boolean haveSetup = AutoSettings.isSaved(blackboard);
        red = settings.alliance() == AutoSettings.Alliance.RED;
        TagLocalizer startFix = new TagLocalizer(START_SAMPLES);
        CsvLog log = new CsvLog("john_teleop", LOG_HEADER);

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
            Pose wall = wallPose(settings);
            startFix.add(camera.poll(), wall.heading());
            Pose start = startPose(wall, startFix);

            telemetry.addData("Alliance", "%s   (B red, X blue)%s", red ? "RED" : "BLUE",
                    haveSetup ? "" : "   (no Game Setup saved)");
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

        Pose start = startPose(wallPose(settings), startFix);
        follower.setPose(start);
        watchdog.poseSet();
        follower.update();
        targetAudience = red;   // match start: red audience / blue far CELL up
        gamepad1.resetEdgeDetection();
        clock.reset();
        stillFor.reset();
        boolean prespin = false;
        boolean holdMode = HOLD_WHEN_RELEASED;
        TagLocalizer relocalizer = new TagLocalizer(100);
        boolean relocalizing = false;
        ElapsedTime relocalizeTimer = new ElapsedTime();
        log.row(0.0, shot, start.x(), start.y(), Math.toDegrees(start.heading()), null, null, null, null, null, null,
                null, null, null, null, upCellName(), watchdog.state(),
                String.format(Locale.US, "start %s at %s (%s)", red ? "red" : "blue", Drawing.format(start),
                        startFix.count() >= MIN_START_SAMPLES ? startFix.count() + " tag readings" : "no tag fix"));

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

            // ---- Camera: only frames taken while standing still ----
            Pose pose = follower.pose();
            Velocity v = follower.velocity();
            boolean nowStill = Math.hypot(v.vx, v.vy) < STILL_SPEED_IPS
                    && Math.abs(v.omega) < Math.toRadians(STILL_TURN_DPS);
            if (nowStill && !still) {
                stillFor.reset();
            }
            still = nowStill;
            HiveCamera.Frame frame = camera.poll();
            if (frame != null && still && stillFor.milliseconds() > frame.ageMs + 30) {
                String change = see(frame, pose);
                if (change != null) {
                    event = change;
                }
                if (relocalizing) {
                    relocalizer.add(frame, pose.heading());
                }
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
                targetAudience = !targetAudience;
                sightings.clear();
                disagreeFrames = 0;
                event = "target switched to the " + upCellName() + " CELL";
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
                    event = startAim();
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

            // ---- Drive (while aiming / shooting the follower holds the aim pose) ----
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

            // ---- Flywheel, indexer, intakes ----
            double[] target = target();
            double distance = Math.hypot(target[0] - pose.x(), target[1] - pose.y());
            if (shot != Shot.FIRE) {
                flywheel.setTargetRpm(rpmFor(distance));
            }
            flywheel.setOn(shot != Shot.IDLE || prespin);
            flywheel.update();
            if (flywheel.event() != null && event == null) {
                event = flywheel.event();
            }
            indexer.update();
            double intake = gamepad1.right_trigger > 0.2 ? INTAKE_POWER : gamepad1.left_trigger > 0.2 ? -INTAKE_POWER : 0.0;
            intakeA.setDirection(INTAKE_A_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeB.setDirection(INTAKE_B_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeA.setPower(intake);
            intakeB.setPower(intake);

            // ---- Telemetry, Dashboard, log ----
            if (watchdog.lost()) {
                telemetry.addLine("ODOMETRY LOST, drive stopped: " + watchdog.reason());
            } else if (!watchdog.ok()) {
                telemetry.addLine("ODOMETRY PAUSED: " + watchdog.reason());
            }
            telemetry.addData("Shot (A)", "%s%s", shot, shotStatus.isEmpty() ? "" : ": " + shotStatus);
            telemetry.addData("Target", "%s %s CELL at (%.1f, %.1f), %.1f in away (%s)", red ? "red" : "blue",
                    upCellName(), target[0], target[1], distance, targetSource());
            telemetry.addData("Flywheel (Y pre-spin)", "%s, %.0f / %.0f RPM%s%s", flywheel.isOn() ? "ON" : "off",
                    flywheel.rpm(), flywheel.targetRpm(), flywheel.atSpeed() ? " AT SPEED" : "",
                    rpmInRange(distance) ? "" : " (distance outside the tested range)");
            telemetry.addData("Pose (FTC)", Drawing.format(pose));
            if (relocalizing) {
                telemetry.addData("Re-localizing", "%d tag readings", relocalizer.count());
            }
            telemetry.addData("Camera", camera.status());
            telemetry.addData("Pinpoint", "%s, %s", state, watchdog.status());
            telemetry.addLine("A aim+shoot, Y pre-spin, B stop, RT/LT intakes, RB slow, dpad up re-localize, dpad down switch CELL");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            Drawing.drawTarget(packet.fieldOverlay(), shot != Shot.IDLE ? pose : null, target[0], target[1], "orange");
            if (aimPose != null && shot != Shot.IDLE) {
                Drawing.drawRobotOutline(packet.fieldOverlay(), aimPose, "orange", null);
            }
            dashboard.sendTelemetryPacket(packet);

            log.row(clock.seconds(), shot, pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                    aimPose == null ? null : Math.toDegrees(aimPose.heading()), target[0], target[1], targetSource(),
                    distance, flywheel.isOn() ? flywheel.targetRpm() : 0.0, flywheel.rpm(), flywheel.mode(),
                    indexer.stepCount(), frame == null ? null : frame.ids(), upCellName(), state, event);
        }
        flywheel.stop();
        indexer.stop();
        intakeA.setPower(0);
        intakeB.setPower(0);
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

    /** The wall pose with Y from the tags (kept on the field), or the wall pose without a tag fix. */
    private static Pose startPose(Pose wall, TagLocalizer fix) {
        if (fix.count() < MIN_START_SAMPLES) {
            return wall;
        }
        double limit = 72.0 - RobotGeometry.LENGTH_IN / 2.0;
        return new Pose(wall.x(), Math.max(-limit, Math.min(limit, fix.y())), wall.heading());
    }

    // =========================================================================
    // Target CELL
    // =========================================================================

    /** Updates the up CELL and its sightings from a still frame; returns an event if the up CELL changed. */
    private String see(HiveCamera.Frame frame, Pose pose) {
        TagLocalizer.UpCell cell = TagLocalizer.upCell(frame, pose, red);
        if (cell == null) {
            return null;
        }
        String event = null;
        if (cell.audience != targetAudience) {
            if (++disagreeFrames >= UP_CELL_FRAMES) {
                targetAudience = cell.audience;
                sightings.clear();
                disagreeFrames = 0;
                event = "the camera sees the " + upCellName() + " CELL up";
            } else {
                return null;
            }
        } else {
            disagreeFrames = 0;
        }
        if (cell.tags > 0) {
            sightings.addLast(new double[]{clock.milliseconds(), cell.x, cell.y});
            while (sightings.size() > 20) {
                sightings.removeFirst();
            }
        }
        return event;
    }

    /** Target {x, y}: the mean of the recent camera sightings, or the up CELL's nominal position. */
    private double[] target() {
        double now = clock.milliseconds();
        double sx = 0, sy = 0;
        int n = 0;
        for (double[] s : sightings) {
            if (now - s[0] <= VISION_MAX_AGE_MS) {
                sx += s[1];
                sy += s[2];
                n++;
            }
        }
        if (n > 0) {
            return new double[]{sx / n, sy / n};
        }
        return new double[]{HiveTags.cellX(targetAudience, true), HiveTags.hiveY(red)};
    }

    private int recentSightings() {
        double now = clock.milliseconds();
        int n = 0;
        for (double[] s : sightings) {
            if (now - s[0] <= VISION_MAX_AGE_MS) n++;
        }
        return n;
    }

    private String targetSource() {
        int n = recentSightings();
        return n > 0 ? "camera, " + n + " frames" : "nominal";
    }

    private String upCellName() {
        return targetAudience ? "audience" : "far";
    }

    // =========================================================================
    // Aim and shoot
    // =========================================================================

    /** Heading that points the shooter from the robot position at the target (plus the trim). */
    private double aimHeading(Pose from) {
        double[] t = target();
        double bearing = Math.atan2(t[1] - from.y(), t[0] - from.x());
        return normalize(RobotGeometry.headingToShoot(bearing) + Math.toRadians(AIM_TRIM_DEG));
    }

    private String startAim() {
        Pose here = follower.pose();
        aimPose = new Pose(here.x(), here.y(), aimHeading(here));
        follower.hold(aimPose);
        shot = Shot.AIM;
        refines = 0;
        alignedSinceMs = -1;
        aimTimer.reset();
        shotStatus = "turning";
        return String.format(Locale.US, "aim at %.1f deg (%s)", Math.toDegrees(aimPose.heading()), targetSource());
    }

    private void cancelShot(String why) {
        if (shot != Shot.IDLE) {
            follower.manual(DrivePowers.zero());
        }
        shot = Shot.IDLE;
        aimPose = null;
        shotStatus = why;
    }

    /** Runs the aim / volley; returns an event or null. */
    private String updateShot() {
        Pose pose = follower.pose();
        switch (shot) {
            case AIM: {
                double error = Math.abs(normalize(aimPose.heading() - pose.heading()));
                boolean aligned = error <= Math.toRadians(AIM_TOLERANCE_DEG) && still;
                double now = aimTimer.milliseconds();
                if (!aligned) {
                    alignedSinceMs = -1;
                } else if (alignedSinceMs < 0) {
                    alignedSinceMs = now;
                }
                if (aligned && now - alignedSinceMs >= AIM_SETTLE_MS) {
                    if (camera.available() && recentSightings() == 0 && now - alignedSinceMs < AIM_SETTLE_MS + VISION_WAIT_MS) {
                        shotStatus = "aimed at the nominal CELL, waiting for the camera";
                        return null;
                    }
                    // The camera now looks at the CELL: re-aim if it moved the target
                    double newAim = aimHeading(new Pose(aimPose.x(), aimPose.y(), pose.heading()));
                    double change = Math.abs(normalize(newAim - aimPose.heading()));
                    if (change > Math.toRadians(AIM_TOLERANCE_DEG) / 2.0 && refines < MAX_REFINES) {
                        refines++;
                        aimPose = aimPose.withHeading(newAim);
                        follower.hold(aimPose);
                        alignedSinceMs = -1;
                        shotStatus = "refining (" + targetSource() + ")";
                        return String.format(Locale.US, "refine %d: aim %.1f deg (%.1f deg change)", refines,
                                Math.toDegrees(newAim), Math.toDegrees(change));
                    }
                    return startFire(String.format(Locale.US, "aimed (%.1f deg off, %s)", Math.toDegrees(error),
                            targetSource()));
                }
                if (now >= AIM_TIMEOUT_MS) {
                    if (error <= Math.toRadians(AIM_MAX_ERROR_DEG)) {
                        return startFire(String.format(Locale.US, "aim timeout, firing %.1f deg off", Math.toDegrees(error)));
                    }
                    cancelShot(String.format(Locale.US, "could not aim (%.1f deg off)", Math.toDegrees(error)));
                    return shotStatus;
                }
                return null;
            }
            case FIRE:
                return updateFeed();
            default:
                return null;
        }
    }

    private String startFire(String why) {
        shot = Shot.FIRE;
        feed = Feed.SPIN_UP;
        stepsDone = 0;
        shotStatus = "waiting for the flywheel";
        return why;
    }

    /** One ball per indexer step, paced for the flywheel (as in the Mechanism Test). */
    private String updateFeed() {
        boolean ready = flywheel.closedLoop() ? flywheel.atSpeed() : flywheel.onSeconds() * 1000.0 >= OPEN_LOOP_SPIN_UP_MS;
        int total = Shooter.INDEXER_STEPS_PER_VOLLEY;
        switch (feed) {
            case SPIN_UP:
                if (ready) {
                    return step();
                }
                return null;
            case STEP:
                // busy() needs a moment to come on after a new target
                if ((feedTimer.milliseconds() > 50 && !indexer.busy()) || feedTimer.milliseconds() > STEP_TIMEOUT_MS) {
                    stepsDone++;
                    if (stepsDone >= total) {
                        cancelShot("volley done");
                        return "volley done";
                    }
                    feed = Feed.INTERVAL;
                    feedTimer.reset();
                }
                return null;
            case INTERVAL:
                if (feedTimer.milliseconds() >= Shooter.SHOT_INTERVAL_MS && ready) {
                    return step();
                }
                return null;
            default:
                return null;
        }
    }

    private String step() {
        indexer.step(1);
        feed = Feed.STEP;
        feedTimer.reset();
        shotStatus = String.format(Locale.US, "shot %d / %d", stepsDone + 1, Shooter.INDEXER_STEPS_PER_VOLLEY);
        return "shot " + (stepsDone + 1);
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /** Flywheel RPM for a distance: linear between the table points, clamped at its ends. */
    private static double rpmFor(double distance) {
        double[] d = parse(RPM_DISTANCES_IN);
        double[] r = parse(RPM_VALUES);
        int n = Math.min(d.length, r.length);
        if (n == 0) {
            return Flywheel.DEFAULT_TARGET_RPM;
        }
        if (distance <= d[0]) {
            return r[0];
        }
        for (int i = 1; i < n; i++) {
            if (distance <= d[i]) {
                return r[i - 1] + (r[i] - r[i - 1]) * (distance - d[i - 1]) / (d[i] - d[i - 1]);
            }
        }
        return r[n - 1];
    }

    private static boolean rpmInRange(double distance) {
        double[] d = parse(RPM_DISTANCES_IN);
        return d.length == 0 || (distance >= d[0] - 3.0 && distance <= d[d.length - 1] + 3.0);
    }

    private static double[] parse(String list) {
        String[] parts = list.split(",");
        double[] out = new double[parts.length];
        int n = 0;
        for (String p : parts) {
            try {
                out[n] = Double.parseDouble(p.trim());
                n++;
            } catch (NumberFormatException ignored) {
                // skip
            }
        }
        double[] trimmed = new double[n];
        System.arraycopy(out, 0, trimmed, 0, n);
        return trimmed;
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
