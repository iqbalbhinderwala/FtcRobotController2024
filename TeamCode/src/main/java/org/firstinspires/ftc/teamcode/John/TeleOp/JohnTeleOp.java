package org.firstinspires.ftc.teamcode.John.TeleOp;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
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
import org.firstinspires.ftc.teamcode.John.Hardware.FallbackDrive;
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
import org.firstinspires.ftc.teamcode.John.Vision.VisionAim;
import org.firstinspires.ftc.teamcode.John.Vision.WallFinder;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * John's TeleOp: field-centric driving, and one button that aims at the up CELL and shoots.
 *
 * START POSITION (chosen in INIT; dpad left / right overrides):
 *  - AUTO: where [John] Auto ended (and which CELL it saw up), when it ended less than
 *    AUTO_POSE_MAX_AGE_S ago and the camera does not disagree: the tags under that heading must
 *    put John within AUTO_CHECK_IN of it, else he was moved since (e.g. carried back after a test)
 *    and WALL is used;
 *  - WALL against either back wall (WallFinder: the tags tell which, no odometry or alliance
 *    needed), else the VISION pose (TagLocalizer.solvePose, median over INIT), else the alliance's
 *    start wall at the Game Setup tile. The autonomous pose is also rejected when the vision
 *    heading differs by more than AUTO_CHECK_DEG. The old description follows:
 *  - WALL: John's camera wall flush on the alliance's start wall (red: the audience wall; blue: the
 *    far wall), shooter facing the field, anywhere along the wall. The camera reads the HIVE tags; a
 *    tag's field Y never changes when the HIVE tips, so each tag gives the position along the wall
 *    (TagLocalizer, median over START_SAMPLES readings). X and heading come from the wall; the X
 *    the tags imply is shown as a check. Without a tag fix, the Game Setup tile (or C1) is used.
 * The alliance comes from [John] Game Setup; only when none was saved B / X in INIT pick it (red /
 * blue). Field-centric driving takes the drivers to be at that alliance's wall.
 *
 * DRIVING: field-centric from the driver's view (red drivers face +Y, blue -Y): pushing the stick
 * away from you drives away from you, whatever way John faces. Pedro's ManualDrive.driveOrHold
 * holds the pose when the sticks are released (HOLD_WHEN_RELEASED).
 *
 * HOLD TO RUN (as on Vex): every automated action runs only while its button is held, step by
 * step; letting go cancels it and gives the robot back to the sticks. Moving a stick also cancels.
 *
 * SHOOT (A held), the same parts as the autonomous:
 *  - Target: our HIVE's up CELL (CellTracker: which one from the tag heights, where from camera
 *    sightings while standing still, else its nominal place). When nobody knows it recently (no
 *    camera confirmation in UP_CELL_TRUST_S, nor the autonomous' at the start), the closest CELL of
 *    our HIVE. If the camera finds the other CELL up while moving or aiming, the shot starts over
 *    for that one (while A is held).
 *  - Outside the shooting zone (ShootingZone: closer than 48", or more than 45 deg to the side of
 *    the way the CELL opening faces), it first drives to the nearest spot inside it, around the
 *    HIVE frame if needed (MOVE_SPEED_IPS), the shooter turning toward the CELL on the way
 *    (Aimer.facingTarget). Blocked or pushed on the way (PathWatcher), it stops.
 *    The flywheel spins up when the shot starts.
 *  - Aimer refines with the camera, then Shooter fires a volley at the speed for the distance (plus
 *    the dpad trim), with the indexer's jam handling. Knocked off aim during the volley, it holds
 *    fire until back on aim; pushed away, it aims again from there (Aimer.keepAim).
 *  - After a volley (or a cancel) it waits for A to be let go before it starts again.
 * AIM ASSIST (right stick button held): you drive, the heading locks onto the CELL.
 *
 * POSITION MODE: ODOMETRY (default: the Pinpoint and the start pose; vision only at the start, for
 * the position along the wall) or VISION-CORRECTED. Back held RELOCALIZE_MS: VISION-CORRECTED (it
 * re-positions from vision first, every time); Start pressed: ODOMETRY. No toggles. Telemetry
 * "Position" shows the mode.
 * RE-POSITION FROM VISION (Back held; anywhere, standing still, HIVE tags in view):
 *  - flush on a back wall: that wall's exact pose (WallFinder);
 *  - else the full pose from the tags alone (TagLocalizer.solvePose, median over the hold). If its
 *    heading agrees with the Pinpoint's within KEEP_HEADING_DEG, the Pinpoint heading is kept (more
 *    exact; it keeps tracking even while John is carried) and only the position comes from the tags;
 *    otherwise the vision heading is taken too.
 * From then on (or from the start with AUTO_CORRECT_FROM_START) VISION CORRECTION runs: whenever
 * John stands still with tags in view, the tags' position goes into a window (CORRECT_WINDOW_MS);
 * every CORRECT_PERIOD_MS the pose moves CORRECT_GAIN of the way to its median, at most
 * CORRECT_MAX_STEP_IN (the tags are noisy: small steps, a median); more than CORRECT_REJECT_IN off
 * is ignored. The heading only beyond CORRECT_HEADING_DEADBAND_DEG, by HEADING_GAIN, at most
 * HEADING_MAX_STEP_DEG per step. Not while a shot runs. Start: back to ODOMETRY mode (the
 * corrections stop; the pose stays where it is).
 *
 * (Earlier description, for the wall part:)
 * RE-LOCALIZE (Back held RELOCALIZE_MS, standing still, the HIVE in view; letting go early cancels):
 *  - against the audience wall (C1 / D1) or the far wall (C6 / D6), either alliance: drive there
 *    and put the camera wall flush on it, shooter facing the field (within WALL_SNAP_IN of the wall
 *    position and WALL_SNAP_DEG of its heading, by the odometry). X and heading from that wall, Y
 *    from the tags of either HIVE (the INIT method; also fixes heading drift). A HIVE's tags face
 *    the end that is up, so from a wall the camera reads the HIVE tilted toward it; if both are
 *    tilted away, use the other wall;
 *  - anywhere else: X and Y from the tags, the heading stays the Pinpoint's.
 * Use it when distances or shooting-zone decisions look off (odometry drift). A shot that misses to
 * one side although the camera sees the CELL is the aim trim or the camera mount, not the pose.
 *
 * gamepad1:
 *   left stick             drive (field-centric)
 *   right stick X          turn;  right stick button (hold): aim assist
 *   A (hold)               shoot: move into range, aim, fire (let go: cancel)
 *   bumpers (press)        intake on / off (toggle; A turns it off);  B (hold): intake reverse while held.
 *                          Driving toward an intake end runs only that intake; standing still (or
 *                          sideways) both
 *   left stick button      robot-centric: switch the stick's forward end (intake B / intake A)
 *   Y                      indexer one step (90 deg) backward, to clear a jam by hand (not while firing)
 *
 * ROBOT_CENTRIC (off by default; field-centric is preferred): the left stick drives relative to the robot, "forward" toward the intake
 * B end or the intake A end (left stick button switches); turning is the same. The automatic shot is unchanged.
 *   Back (hold 1 s)        VISION-CORRECTED: re-position from vision, then keep correcting
 *   Start (press)          ODOMETRY: no vision corrections
 *   dpad up / down         flywheel trim + / - TRIM_STEP_RPM;  dpad left / right: trim 0
 *
 * WITHOUT ODOMETRY (OdometryWatchdog PAUSED or LOST): the sticks drive the motors directly,
 * field-centric on the Control Hub IMU (FallbackDrive, kept aligned with the Pinpoint heading while
 * that is good), so John stays drivable. While only PAUSED a running shot waits for it as before.
 * LOST (for the rest of the OpMode): A held is a VISION SHOT: the camera's view of the up CELL
 * (VisionAim) gives the turn (made on the IMU), the distance (flywheel speed) and the zone check;
 * you drive (hints in telemetry: back up / move toward the middle), the heading locks onto the CELL,
 * and it fires when in range, on aim and the sticks are still. No drive into range (that needs the
 * position). The right stick button locks the heading onto the CELL the same way. Back held against a
 * back wall re-aligns the IMU heading to it.
 *
 * OdometryWatchdog stops Pedro's drive on bad Pinpoint reads. Logged to
 * /sdcard/FIRST/john_logs/r<run>_john_teleop_<time>.csv.
 */
@Config("JohnTeleOp")
@TeleOp(name = "[John] TeleOp", group = "John")
public class JohnTeleOp extends LinearOpMode {

    // ---- Driving ----
    /** Testing: drive relative to the robot instead of the field (left stick button picks the forward end). */
    public static boolean ROBOT_CENTRIC = false;
    public static double DRIVE_SCALE = 0.8;
    public static double TURN_SCALE = 0.6;
    public static double STICK_DEADBAND = 0.05;
    public static boolean HOLD_WHEN_RELEASED = true;
    /** A stick moved this far cancels an aim / volley. */
    public static double CANCEL_STICK = 0.3;
    /** Aim assist: turn power per radian of heading error, at most AIM_ASSIST_MAX_TURN. */
    public static double AIM_ASSIST_KP = 1.5;
    public static double AIM_ASSIST_MAX_TURN = 0.6;
    public static double TRIM_STEP_RPM = 50;
    /** Driving toward an intake end at least this much (stick, robot frame): only that intake runs. */
    public static double INTAKE_LEAD_MIN = 0.15;
    /** After this long in INIT, warn if the camera has not seen a HIVE tag. */
    public static double CAMERA_CHECK_S = 2.0;

    // ---- Without odometry (vision shot) ----
    /** Heading lock on the IMU: turn power per radian, at most FALLBACK_MAX_TURN. */
    public static double FALLBACK_KP = 1.5;
    public static double FALLBACK_MAX_TURN = 0.6;
    /** A camera solution older than this is not used. */
    public static double VISION_MAX_AGE_MS = 500;
    /** Update the aim from the camera only when turning slower than this (frames lag). */
    public static double VISION_STILL_TURN_DPS = 10.0;

    // ---- Start position and re-localizing ----
    /** Start from the autonomous' last pose when it is younger than this (at INIT). */
    public static double AUTO_POSE_MAX_AGE_S = 60.0;
    /** ...and the camera puts John within this of it (else he was moved since). */
    public static double AUTO_CHECK_IN = 6.0;
    /** ...and its heading within this of the vision heading. */
    public static double AUTO_CHECK_DEG = 20.0;
    /** Tag readings kept for the start position (median). */
    public static int START_SAMPLES = 60;
    /** Fewer readings than this: no tag fix, use the Game Setup tile. */
    public static int MIN_START_SAMPLES = 5;
    /** Warn when the tags' X differs from the wall X by more than this (not flush, or mount / tag size off). */
    public static double X_CHECK_WARN_IN = 4.0;
    /** Re-localize: collect tags for this long while X is held and John stands still. */
    public static double RELOCALIZE_MS = 1000;
    /** Re-position: keep the Pinpoint heading when the vision heading is within this of it. */
    public static double KEEP_HEADING_DEG = 10.0;
    /** Full vision pose: at least this many tags spread at least this far (for the heading). */
    public static int POSE_MIN_TAGS = 3;
    public static double POSE_MIN_SPAN_IN = 8.0;

    // ---- Vision correction (after the first re-position) ----
    public static boolean AUTO_CORRECT_FROM_START = false;
    public static double CORRECT_PERIOD_MS = 300;
    public static double CORRECT_WINDOW_MS = 1500;
    public static int CORRECT_MIN_SAMPLES = 5;
    public static double CORRECT_GAIN = 0.15;
    public static double CORRECT_MAX_STEP_IN = 0.5;
    public static double CORRECT_REJECT_IN = 24.0;
    public static double CORRECT_HEADING_DEADBAND_DEG = 3.0;
    public static double HEADING_GAIN = 0.05;
    public static double HEADING_MAX_STEP_DEG = 0.5;
    public static int MIN_RELOCALIZE_SAMPLES = 4;
    /** Re-localize against the audience / far wall when this close to its position and heading. */
    public static double WALL_SNAP_IN = 6.0;
    public static double WALL_SNAP_DEG = 10.0;
    /** The up CELL counts as known this long after the camera (or the autonomous) said which. */
    public static double UP_CELL_TRUST_S = 15.0;

    // ---- Driving into the shooting zone ----
    /**
     * Speed cap for the drive into range; 0 = none (2026-10-24, was 40: with Pedro's speed cap short
     * moves of 2-4" never got going, r0169 "could not reach the shooting spot").
     */
    public static double MOVE_SPEED_IPS = 0.0;
    /** This much too close / too angled still counts as in range (odometry noise at the edge, r0169). */
    public static double ZONE_TOLERANCE_IN = 1.5;
    public static double ZONE_TOLERANCE_DEG = 2.0;
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
        FallbackDrive fallback = new FallbackDrive(hardwareMap);

        AutoSettings settings = new AutoSettings(blackboard);
        boolean haveSetup = AutoSettings.isSaved(blackboard);
        String setupSource = "Game Setup";
        if (!haveSetup) {
            // After an app restart (or a redeploy) the blackboard is empty: the last Game Setup file
            AutoSettings fromFile = AutoSettings.loadFile();
            if (fromFile != null) {
                settings = fromFile;
                haveSetup = true;
                setupSource = "the last Game Setup (file, " + fromFile.fileAge() + ")";
            }
        }
        red = settings.alliance() == AutoSettings.Alliance.RED;
        TagLocalizer startFix = new TagLocalizer(START_SAMPLES);
        CsvLog log = new CsvLog("john_teleop", LOG_HEADER);
        Pose autoPose = AutoSettings.loadPose(blackboard);
        double autoAgeS = AutoSettings.poseAgeS(blackboard);
        Boolean autoUpAudience = AutoSettings.loadUpAudience(blackboard);
        // An impossible pose (off the field, or inside the HIVE frame, e.g. the Pinpoint's (0, 0, 0))
        // is never used
        boolean autoPossible = autoPose != null && Math.abs(autoPose.x()) < 72.0 && Math.abs(autoPose.y()) < 72.0
                && !(Math.abs(autoPose.x()) < ShootingZone.HIVE_HALF_X && Math.abs(autoPose.y()) < ShootingZone.HIVE_HALF_Y);
        boolean autoFresh = autoPossible && autoAgeS <= AUTO_POSE_MAX_AGE_S;
        Start startFrom = autoFresh ? Start.AUTO : Start.WALL;
        boolean startOverridden = false;
        WallFinder walls = new WallFinder(START_SAMPLES);
        TagLocalizer autoCheck = new TagLocalizer(START_SAMPLES);
        List<Pose> initVision = new ArrayList<>();   // full poses from the tags alone

        // ---- INIT: nothing moves; the camera finds the position along the wall ----
        gamepad1.resetEdgeDetection();
        ElapsedTime initTimer = new ElapsedTime();
        int initFrames = 0;
        int initFramesWithTags = 0;
        while (opModeInInit()) {
            // The alliance comes from Game Setup; B / X only without one (B is "intake reverse" in the
            // TeleOp, and pressed by habit in INIT it flipped the alliance: r0132)
            boolean pickRed = gamepad1.bWasPressed(), pickBlue = gamepad1.xWasPressed();
            if (!haveSetup && pickRed && !red) {
                red = true;
                startFix.reset();
            }
            if (!haveSetup && pickBlue && red) {
                red = false;
                startFix.reset();
            }
            if ((gamepad1.dpadLeftWasPressed() | gamepad1.dpadRightWasPressed()) && autoPose != null) {
                startFrom = startFrom == Start.AUTO ? Start.WALL : Start.AUTO;
                startOverridden = true;
            }
            Pose wall = wallPose(settings);
            HiveCamera.Frame initFrame = camera.poll();
            if (initFrame != null) {
                initFrames++;
                if (!initFrame.tags.isEmpty()) {
                    initFramesWithTags++;
                }
            }
            if (!camera.available()) {
                telemetry.addLine("!!! NO CAMERA: " + camera.status());
            } else if (initTimer.seconds() >= CAMERA_CHECK_S && initFrames == 0) {
                telemetry.addLine("!!! THE LIMELIGHT SENDS NO FRAMES (pipeline " + HiveCamera.PIPELINE + " running? cable?)");
            } else if (initTimer.seconds() >= CAMERA_CHECK_S && initFramesWithTags == 0) {
                telemetry.addLine(String.format(Locale.US, "!!! THE CAMERA SEES NO HIVE TAGS in %d frames: the start pose "
                        + "is a guess. Check the Limelight pipeline %d (AprilTag 36h11) / exposure / gain / lens, and "
                        + "that John faces the HIVE", initFrames, HiveCamera.PIPELINE));
            }
            startFix.add(initFrame, wall.heading());
            walls.add(initFrame);
            if (autoPose != null) {
                autoCheck.add(initFrame, autoPose.heading());
            }
            Pose seenPose = TagLocalizer.solvePose(initFrame, POSE_MIN_TAGS, POSE_MIN_SPAN_IN);
            if (seenPose != null) {
                initVision.add(seenPose);
                if (initVision.size() > START_SAMPLES) {
                    initVision.remove(0);
                }
            }
            Pose visionStart = initVision.size() >= MIN_START_SAMPLES ? medianPose(initVision) : null;
            double autoError = autoPose != null && autoCheck.count() >= MIN_START_SAMPLES
                    ? Math.hypot(autoCheck.x() - autoPose.x(), autoCheck.y() - autoPose.y()) : Double.NaN;
            double autoHeadingError = autoPose != null && visionStart != null
                    ? Math.abs(Math.toDegrees(normalize(visionStart.heading() - autoPose.heading()))) : Double.NaN;
            boolean autoOk = autoFresh && (Double.isNaN(autoError) || autoError <= AUTO_CHECK_IN)
                    && (Double.isNaN(autoHeadingError) || autoHeadingError <= AUTO_CHECK_DEG);
            if (!startOverridden) {
                startFrom = autoOk ? Start.AUTO : Start.WALL;
            }
            Pose wallFound = walls.found(MIN_START_SAMPLES);
            Pose start = startFrom == Start.AUTO ? autoPose
                    : wallFound != null ? wallFound
                    : visionStart != null ? visionStart
                    : startFix.alongWall(wall, MIN_START_SAMPLES, RobotGeometry.LENGTH_IN / 2.0);

            telemetry.addData("Alliance", haveSetup ? "%s   (from %s; drivers at the %s wall)"
                            : "%s   (%sno Game Setup saved: B red, X blue; drivers at the %s wall)",
                    red ? "RED" : "BLUE", haveSetup ? setupSource : "", red ? "red" : "blue");
            telemetry.addData("Start from", "%s   (dpad left / right)%s", startFrom,
                    autoPose == null ? ": no autonomous pose"
                            : !autoPossible ? ": the autonomous pose " + Drawing.format(autoPose) + " is impossible, not used"
                            : String.format(Locale.US, ": the autonomous ended %.0f s ago%s%s",
                            autoAgeS, autoAgeS > AUTO_POSE_MAX_AGE_S ? " (old)" : "",
                            Double.isNaN(autoError) ? ", camera cannot check it"
                                    : String.format(Locale.US, ", camera: %.1f in%s from it%s", autoError,
                                            Double.isNaN(autoHeadingError) ? ""
                                                    : String.format(Locale.US, " / %.0f deg", autoHeadingError),
                                            autoOk ? "" : " -> MOVED SINCE, not used")));
            telemetry.addData("Back wall (camera)", walls.describe(MIN_START_SAMPLES));
            telemetry.addData("Vision pose (camera)", visionStart == null
                    ? "none (" + initVision.size() + " frames with enough tags)"
                    : Drawing.format(visionStart) + " (" + initVision.size() + " frames)");
            if (startFrom == Start.WALL && wallFound == null && visionStart == null) {
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
            Drawing.drawTags(packet.fieldOverlay(), start, initFrame);
            dashboard.sendTelemetryPacket(packet);
        }
        if (isStopRequested()) {
            camera.stop();
            log.close();
            watchdog.close();
            return;
        }

        Pose wallFound = walls.found(MIN_START_SAMPLES);
        Pose visionStart = initVision.size() >= MIN_START_SAMPLES ? medianPose(initVision) : null;
        Pose start = startFrom == Start.AUTO ? autoPose
                : wallFound != null ? wallFound
                : visionStart != null ? visionStart
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
        boolean shotLatched = false;      // A must be let go before the next shot
        boolean upFromAuto = startFrom == Start.AUTO && autoUpAudience != null;
        TagLocalizer relocalizer = new TagLocalizer(100);
        WallFinder relocalizeWalls = new WallFinder(100);
        List<Pose> relocalizeVision = new ArrayList<>();
        boolean autoCorrect = AUTO_CORRECT_FROM_START;
        ArrayDeque<double[]> correctWindow = new ArrayDeque<>();   // {ms, x, y, heading or NaN}
        ElapsedTime correctTimer = new ElapsedTime();
        double lastRejectLogMs = -1e9;
        double correctedIn = 0.0;   // total correction, for telemetry
        boolean relocalizing = false;
        boolean relocalizeDone = false;   // Back must be let go before the next toggle
        boolean relocalizeTurnsOff = false;   // this hold switches back to ODOMETRY mode
        Pose relocalizeWall = null;       // the wall pose when re-localizing against a wall
        ElapsedTime relocalizeTimer = new ElapsedTime();
        boolean holdMode = HOLD_WHEN_RELEASED;
        boolean intakeOn = false;
        boolean forwardIsB = true;
        // Without odometry
        boolean visionShot = false;
        boolean visionVolleyDone = false;
        boolean wasFiring = false;
        VisionAim lastAim = null;
        double lastAimMs = -1e9;
        double visionTarget = Double.NaN;   // field heading to hold (IMU), from the camera
        log.row(0.0, shot, start.x(), start.y(), Math.toDegrees(start.heading()), null, null, null, null, null, null,
                null, null, null, null, tracker.upCellName(), watchdog.state(),
                String.format(Locale.US, "start %s at %s (%s)", red ? "red" : "blue", Drawing.format(start),
                        startFrom == Start.AUTO ? "from the autonomous"
                                : wallFound != null ? walls.describe(MIN_START_SAMPLES)
                                : visionStart != null ? "vision pose from " + initVision.size() + " frames"
                                : startFix.count() >= MIN_START_SAMPLES ? startFix.count() + " tag readings" : "no tag fix"));

        while (opModeIsActive()) {
            String event = null;
            // Held buttons run the automated actions; pressed (edge) buttons toggle / step
            boolean shootHeld = gamepad1.a;
            boolean relocalizeHeld = gamepad1.back;
            if (gamepad1.startWasPressed() && autoCorrect) {
                autoCorrect = false;
                correctWindow.clear();
                event = "position mode: ODOMETRY (Start; vision corrections off)";
            }
            boolean assistHeld = gamepad1.right_stick_button;
            boolean intakeToggle = gamepad1.leftBumperWasPressed() | gamepad1.rightBumperWasPressed();
            if (gamepad1.yWasPressed()) {
                if (shooter.firing()) {
                    event = "indexer back: not while firing";
                } else {
                    shooter.indexer().step(-1);   // also clears a STUCK indexer
                    event = "indexer one step back (unclog)";
                }
            }
            if (gamepad1.leftStickButtonWasPressed()) {
                forwardIsB = !forwardIsB;
                event = "forward = intake " + (forwardIsB ? "B" : "A");
            }
            double ends = forwardIsB ? 1.0 : -1.0;   // robot +X is the intake B end
            boolean trimUp = gamepad1.dpadUpWasPressed(), trimDown = gamepad1.dpadDownWasPressed();
            boolean trimReset = gamepad1.dpadLeftWasPressed() | gamepad1.dpadRightWasPressed();

            double stickFwd = deadband(-gamepad1.left_stick_y);
            double stickLeft = deadband(-gamepad1.left_stick_x);
            double stickTurn = deadband(-gamepad1.right_stick_x);
            boolean sticksMoved = Math.max(Math.abs(stickFwd), Math.max(Math.abs(stickLeft), Math.abs(stickTurn)))
                    > CANCEL_STICK;

            // ---- Odometry state (from the last loop's check) ----
            boolean lost = watchdog.lost();
            boolean degraded = lost || (!watchdog.ok() && shot == Shot.IDLE);
            fallback.update();

            // ---- Camera: the tracker uses only frames taken standing still ----
            Pose pose = follower.pose();
            HiveCamera.Frame frame = camera.poll();
            boolean imuStill = Math.abs(fallback.turnRate()) < Math.toRadians(VISION_STILL_TURN_DPS);
            boolean usable = lost
                    ? tracker.update(frame, new Pose(lastAim == null ? 0.0 : lastAim.robotX,
                            lastAim == null ? 0.0 : lastAim.robotY, fallback.heading()),
                            imuStill && Math.abs(stickFwd) + Math.abs(stickLeft) < 0.05)
                    : tracker.update(frame, follower);
            if (usable && relocalizing && !lost) {
                relocalizer.add(frame, pose.heading());
                relocalizeWalls.add(frame);
                Pose seen = TagLocalizer.solvePose(frame, POSE_MIN_TAGS, POSE_MIN_SPAN_IN);
                if (seen != null) {
                    relocalizeVision.add(seen);
                }
            }
            if (usable && autoCorrect && !lost && !relocalizing && shot == Shot.IDLE && frame != null
                    && !frame.tags.isEmpty()) {
                // This frame's position with the Pinpoint heading (median over its tags) + its own heading
                List<Double> xs = new ArrayList<>(), ys = new ArrayList<>();
                for (HiveCamera.Tag tag : frame.tags) {
                    double[] xy = TagLocalizer.robotPosition(tag, pose.heading());
                    xs.add(xy[0]);
                    ys.add(xy[1]);
                }
                Pose seen = TagLocalizer.solvePose(frame, POSE_MIN_TAGS, POSE_MIN_SPAN_IN);
                correctWindow.addLast(new double[]{clock.milliseconds(), median(xs), median(ys),
                        seen == null ? Double.NaN : seen.heading()});
            }
            while (!correctWindow.isEmpty() && clock.milliseconds() - correctWindow.peekFirst()[0] > CORRECT_WINDOW_MS) {
                correctWindow.removeFirst();
            }
            if (!tracker.still()) {
                correctWindow.clear();   // samples are only good for where John stood
            }
            if (tracker.event() != null) {
                event = tracker.event();
                if (shot == Shot.MOVE || shot == Shot.AIM) {
                    // The other CELL is up: start the shot over for it (A is still held)
                    cancelShot("the " + tracker.upCellName() + " CELL is up");
                    event = event + "; " + requestShot();
                }
            }

            // ---- Buttons ----
            if (trimUp || trimDown || trimReset) {
                shooter.setTrimRpm(trimReset ? 0.0 : shooter.trimRpm() + (trimUp ? TRIM_STEP_RPM : -TRIM_STEP_RPM));
                event = String.format(Locale.US, "flywheel trim %+.0f RPM", shooter.trimRpm());
            }
            // Re-localize: runs while X is held, done after RELOCALIZE_MS
            if (relocalizeHeld && !relocalizing && !relocalizeDone && shot == Shot.IDLE && lost) {
                // No odometry: only the heading, against a back wall (+90 audience / -90 far)
                relocalizing = true;
                relocalizeTimer.reset();
                double h = fallback.heading();
                double wallHeading = Math.toRadians(Math.cos(h - Math.PI / 2) >= 0 ? 90.0 : -90.0);
                boolean atWall = Math.abs(Math.toDegrees(normalize(h - wallHeading))) <= WALL_SNAP_DEG;
                relocalizeWall = atWall ? new Pose(0.0, 0.0, wallHeading) : null;
                event = atWall ? "re-align the IMU heading to the wall: keep Back held"
                        : "no odometry: re-align only with the camera wall flush on a back wall";
            }
            if (relocalizeHeld && !relocalizing && !relocalizeDone && shot == Shot.IDLE && !lost) {
                relocalizing = true;
                relocalizer.reset();
                relocalizeWalls.reset();
                relocalizeVision.clear();
                relocalizeTimer.reset();
                relocalizeTurnsOff = false;
                event = "re-position from vision: keep Back held, stand still";
            }
            if (relocalizing && !relocalizeHeld) {
                relocalizing = false;
                event = "position mode unchanged: Back let go";
            }
            if (!relocalizeHeld) {
                relocalizeDone = false;
            }
            // Shoot: runs while A is held; a new one only after letting go
            if (lost) {
                if (shootHeld && !shotLatched) {
                    shotLatched = true;
                    intakeOn = false;   // a shot stops the intake
                    visionShot = true;
                    visionVolleyDone = false;
                    visionTarget = Double.NaN;
                    event = "vision shot (no odometry): drive into range, the heading locks onto the CELL";
                }
                if (!shootHeld && visionShot) {
                    shooter.cancel();
                    shooter.setPaused(false);
                    visionShot = false;
                    event = "vision shot cancelled: A let go";
                }
            }
            if (shootHeld && shot == Shot.IDLE && !shotLatched && !lost) {
                shotLatched = true;
                intakeOn = false;   // a shot stops the intake
                if (!watchdog.ok()) {
                    shotStatus = "can't aim: odometry " + watchdog.state();
                    event = shotStatus;
                } else {
                    String pick = pickCellIfUnknown(upFromAuto, clock.seconds());
                    event = (pick == null ? "" : pick + "; ") + requestShot();
                }
            }
            if (!shootHeld) {
                if (shot != Shot.IDLE) {
                    cancelShot("cancelled: A let go");
                    event = "shot cancelled: A let go";
                }
                shotLatched = false;
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
            if (relocalizing && lost) {
                if (sticksMoved) {
                    relocalizing = false;
                    event = "re-align cancelled: moved";
                } else if (relocalizeTimer.milliseconds() >= RELOCALIZE_MS) {
                    relocalizing = false;
                    relocalizeDone = true;
                    if (relocalizeWall != null) {
                        double before = fallback.heading();
                        fallback.alignTo(relocalizeWall.heading());
                        event = String.format(Locale.US, "IMU heading re-aligned to the wall: %.1f -> %.1f deg",
                                Math.toDegrees(before), Math.toDegrees(relocalizeWall.heading()));
                    }
                }
            }
            if (relocalizing && !lost) {
                if (sticksMoved) {
                    relocalizing = false;
                    event = "re-localize cancelled: moved";
                } else if (relocalizeTimer.milliseconds() >= RELOCALIZE_MS && relocalizeTurnsOff) {
                    relocalizing = false;
                    relocalizeDone = true;   // once per hold
                    autoCorrect = false;
                    correctWindow.clear();
                    event = "position mode: ODOMETRY (vision corrections off)";
                } else if (relocalizeTimer.milliseconds() >= RELOCALIZE_MS) {
                    relocalizing = false;
                    relocalizeDone = true;   // once per hold
                    // Against a back wall: exact. Else the vision pose (keeping the Pinpoint heading
                    // when it agrees), else the tags with the Pinpoint heading.
                    Pose before = follower.pose();
                    Pose atWall = relocalizeWalls.found(MIN_RELOCALIZE_SAMPLES);
                    Pose vision = relocalizeVision.size() >= MIN_RELOCALIZE_SAMPLES ? medianPose(relocalizeVision) : null;
                    Pose fixed = null;
                    String how = null;
                    if (atWall != null) {
                        fixed = atWall;
                        how = atWall.x() > 0 ? "audience wall" : "far wall";
                    } else if (vision != null) {
                        double dh = Math.toDegrees(normalize(vision.heading() - before.heading()));
                        if (Math.abs(dh) <= KEEP_HEADING_DEG) {
                            fixed = relocalizer.count() >= MIN_RELOCALIZE_SAMPLES
                                    ? new Pose(relocalizer.x(), relocalizer.y(), before.heading())
                                    : new Pose(vision.x(), vision.y(), before.heading());
                            how = String.format(Locale.US, "vision, Pinpoint heading kept (vision %+.1f deg)", dh);
                        } else {
                            fixed = vision;
                            how = String.format(Locale.US, "vision, heading too (%+.1f deg)", dh);
                        }
                    } else if (relocalizer.count() >= MIN_RELOCALIZE_SAMPLES) {
                        fixed = new Pose(relocalizer.x(), relocalizer.y(), before.heading());
                        how = "tags, Pinpoint heading (too few tags for a vision heading)";
                    }
                    if (fixed != null) {
                        follower.setPose(fixed);
                        watchdog.poseSet();
                        follower.manual(DrivePowers.zero());   // drop a hold in the old coordinates
                        fallback.alignTo(fixed.heading());
                        autoCorrect = true;
                        correctWindow.clear();
                        event = String.format(Locale.US, "re-positioned (%s, %d frames): %s -> %s; vision correction on",
                                how, relocalizeVision.size(), Drawing.format(before), Drawing.format(fixed));
                    } else {
                        event = "re-localize failed: " + relocalizer.count() + " tag readings (standing still? HIVE in "
                                + "view? against a back wall, both HIVEs may be tilted away from it: try the other wall)";
                    }
                }
            }

            // ---- Vision correction: small steps toward the median of the recent tag positions ----
            if (autoCorrect && !lost && watchdog.ok() && !relocalizing && shot == Shot.IDLE && tracker.still()
                    && correctTimer.milliseconds() >= CORRECT_PERIOD_MS && correctWindow.size() >= CORRECT_MIN_SAMPLES) {
                correctTimer.reset();
                List<Double> xs = new ArrayList<>(), ys = new ArrayList<>(), hs = new ArrayList<>();
                Pose now = follower.pose();
                for (double[] w : correctWindow) {
                    xs.add(w[1]);
                    ys.add(w[2]);
                    if (!Double.isNaN(w[3])) {
                        hs.add(normalize(w[3] - now.heading()));
                    }
                }
                double ex = median(xs) - now.x();
                double ey = median(ys) - now.y();
                double off = Math.hypot(ex, ey);
                if (off > CORRECT_REJECT_IN) {
                    if (clock.milliseconds() - lastRejectLogMs > 5000) {
                        lastRejectLogMs = clock.milliseconds();
                        event = String.format(Locale.US, "vision disagrees by %.0f in: not corrected (Start: odometry only)", off);
                    }
                } else {
                    double scale = Math.min(1.0, CORRECT_MAX_STEP_IN / Math.max(1e-9, CORRECT_GAIN * off));
                    double sx = CORRECT_GAIN * ex * scale;
                    double sy = CORRECT_GAIN * ey * scale;
                    double sh = 0.0;
                    if (hs.size() >= CORRECT_MIN_SAMPLES) {
                        double dh = median(hs);
                        if (Math.abs(Math.toDegrees(dh)) > CORRECT_HEADING_DEADBAND_DEG) {
                            double maxStep = Math.toRadians(HEADING_MAX_STEP_DEG);
                            sh = Math.max(-maxStep, Math.min(maxStep, HEADING_GAIN * dh));
                        }
                    }
                    if (Math.hypot(sx, sy) > 0.02 || sh != 0.0) {
                        follower.setPose(new Pose(now.x() + sx, now.y() + sy, normalize(now.heading() + sh)));
                        watchdog.poseSet();
                        follower.manual(DrivePowers.zero());   // re-hold at the corrected pose
                        correctedIn += Math.hypot(sx, sy);
                    }
                }
            }

            // ---- Aim and shoot ----
            String shotEvent = updateShot();
            if (shotEvent != null) {
                event = shotEvent;
            }

            // ---- Drive (while moving / aiming / shooting the follower drives itself) ----
            if (degraded && shot == Shot.IDLE) {
                // Pedro gets no command meanwhile, so it does not drive back to an old hold on RESUMED
                follower.manual(DrivePowers.zero());
            }
            if (shot == Shot.IDLE && !degraded) {
                DrivePowers powers = ROBOT_CENTRIC
                        ? new DrivePowers(ends * stickFwd * DRIVE_SCALE, ends * stickLeft * DRIVE_SCALE, stickTurn * TURN_SCALE)
                        : ManualDrive.fieldCentric(stickFwd * DRIVE_SCALE,
                                stickLeft * DRIVE_SCALE, stickTurn * TURN_SCALE,
                                pose.heading(), -driverFacing());   // Pedro turns the sticks by -(heading + offset)
                if (assistHeld) {
                    // Aim assist: the driver drives, the heading locks onto the CELL
                    DrivePowers locked = ManualDrive.headingLock(follower,
                            Controller.proportional(AIM_ASSIST_KP), powers, aimer.aimHeading(pose));
                    double turn = Math.max(-AIM_ASSIST_MAX_TURN, Math.min(AIM_ASSIST_MAX_TURN, locked.turn()));
                    follower.manual(new DrivePowers(locked.forward(), locked.strafe(), turn));
                } else if (holdMode) {
                    ManualDrive.driveOrHold(follower, powers);
                } else {
                    follower.manual(powers);
                }
            }
            follower.update();
            OdometryWatchdog.State state = watchdog.check(follower);
            pose = follower.pose();
            if (watchdog.ok()) {
                fallback.alignTo(pose.heading());   // the IMU carries on from here if the Pinpoint fails
            }

            // ---- Without odometry: drive the motors directly, field-centric on the IMU ----
            String visionStatus = null;
            if (degraded) {
                if (lost && (visionShot || assistHeld)) {
                    VisionAim aim = VisionAim.solve(frame, red, tracker.upAudience(), fallback.heading());
                    if (aim != null) {
                        lastAim = aim;
                        lastAimMs = clock.milliseconds();
                        if (imuStill || Double.isNaN(visionTarget)) {
                            visionTarget = normalize(fallback.heading() + aim.turn);
                        }
                    }
                }
                boolean fresh = lastAim != null && clock.milliseconds() - lastAimMs <= VISION_MAX_AGE_MS;
                double turn = stickTurn * TURN_SCALE;
                double error = Double.NaN;
                if (lost && (visionShot || assistHeld) && !Double.isNaN(visionTarget)) {
                    error = normalize(visionTarget - fallback.heading());
                    turn = Math.max(-FALLBACK_MAX_TURN, Math.min(FALLBACK_MAX_TURN, FALLBACK_KP * error));
                }
                if (ROBOT_CENTRIC) {
                    fallback.drive(ends * stickFwd * DRIVE_SCALE, ends * stickLeft * DRIVE_SCALE, turn);
                } else {
                    fallback.driveFieldCentric(stickFwd * DRIVE_SCALE, stickLeft * DRIVE_SCALE, turn, driverFacing());
                }

                if (lost && visionShot) {
                    String problem = fresh ? lastAim.zoneProblem() : "the camera does not see the "
                            + tracker.upCellName() + " CELL: turn toward it";
                    boolean onAim = !Double.isNaN(error) && Math.abs(error) <= Math.toRadians(Aimer.FIRE_TOLERANCE_DEG);
                    boolean sticksStill = Math.abs(stickFwd) + Math.abs(stickLeft) < 0.05;
                    if (fresh) {
                        shooter.setDistance(lastAim.distance);
                    }
                    if (problem == null && onAim && sticksStill && !shooter.firing() && !visionVolleyDone) {
                        shooter.startVolley();
                        event = String.format(Locale.US, "vision volley at the %s CELL from %.0f in", tracker.upCellName(),
                                lastAim.distance);
                    }
                    shooter.setPaused(!(onAim && sticksStill));
                    if (wasFiring && !shooter.firing()) {
                        visionVolleyDone = true;
                        shooter.setPaused(false);
                    }
                    visionStatus = visionVolleyDone ? "volley done (let go of A)"
                            : problem != null ? problem + (fresh && lastAim.hint() != null ? ": " + lastAim.hint() : "")
                            : !onAim ? String.format(Locale.US, "turning (%.1f deg off)", Math.toDegrees(Math.abs(error)))
                            : !sticksStill ? "in range: stop driving to fire" : shooter.firing() ? "firing" : "ready";
                }
            }
            wasFiring = shooter.firing();

            // ---- Shooter, intakes ----
            double[] target = tracker.target();
            double distance = Math.hypot(target[0] - pose.x(), target[1] - pose.y());
            if (!lost) {
                shooter.setDistance(distance);
            }
            shooter.setSpin(shot != Shot.IDLE || visionShot);
            String shooterEvent = shooter.update();
            if (shooterEvent != null) {
                event = event == null ? shooterEvent : event + "; " + shooterEvent;
            }
            if (intakeToggle) {
                intakeOn = !intakeOn;
                event = event == null ? (intakeOn ? "intakes on" : "intakes off") : event;
            }
            // Driving toward an intake end: only that intake (the one meeting the balls); standing
            // still or sideways: both. B held reverses (e.g. to clear a ball).
            double robotForward;
            if (ROBOT_CENTRIC) {
                robotForward = ends * stickFwd;
            } else {
                double a = driverFacing() - (degraded ? fallback.heading() : pose.heading());
                robotForward = stickFwd * Math.cos(a) - stickLeft * Math.sin(a);
            }
            double intakePower = gamepad1.b ? -Intake.POWER : intakeOn ? Intake.POWER : 0.0;
            if (Math.abs(robotForward) >= INTAKE_LEAD_MIN) {
                intake.only(robotForward > 0 ? Intake.Side.B : Intake.Side.A, intakePower);   // +X is the B end
            } else {
                intake.set(intakePower);
            }

            // ---- Telemetry, Dashboard, log ----
            if (watchdog.lost()) {
                telemetry.addLine("NO ODOMETRY (LOST): driving on the IMU; A = vision shot. " + watchdog.reason());
                telemetry.addData("Vision shot (hold A)", visionShot ? (visionStatus == null ? "..." : visionStatus) : "off");
            } else if (!watchdog.ok()) {
                telemetry.addLine("ODOMETRY PAUSED: driving on the IMU meanwhile. " + watchdog.reason());
            }
            telemetry.addData("Position", autoCorrect
                    ? String.format(Locale.US, "VISION-CORRECTED (%d samples, %.1f in corrected so far; Start: odometry)",
                            correctWindow.size(), correctedIn)
                    : "ODOMETRY (hold Back 1 s: re-position from vision)");
            telemetry.addData("Shot (hold A)", "%s%s", shot, shotStatus.isEmpty() ? "" : ": " + shotStatus);
            telemetry.addData("Target", "%s %s CELL at (%.1f, %.1f), %.1f in away (%s)", red ? "red" : "blue",
                    tracker.upCellName(), target[0], target[1], distance, tracker.source());
            telemetry.addData("Shooter", "%s, trim %+.0f (dpad)", shooter.status(), shooter.trimRpm());
            telemetry.addData("Intakes (bumper toggles, B reverse)", intake.status());
            telemetry.addData("Drive", ROBOT_CENTRIC ? "ROBOT-centric, forward = intake " + (forwardIsB ? "B" : "A")
                    + " (left stick button switches)" : "field-centric");
            String zone = ShootingZone.problem(pose.x(), pose.y(), target[0], target[1], tracker.upAudience(),
                    ZONE_TOLERANCE_IN, ZONE_TOLERANCE_DEG);
            telemetry.addData("Shooting zone", zone == null
                    ? String.format(Locale.US, "IN RANGE (%.0f deg off the CELL axis)",
                            ShootingZone.offAxisDeg(pose.x(), pose.y(), target[0], target[1], tracker.upAudience()))
                    : zone + ": the shot drives into range first");
            telemetry.addData("Pose (FTC)", Drawing.format(pose));
            if (relocalizing) {
                telemetry.addData("Re-positioning", "%d tag readings, %d vision poses; %s", relocalizer.count(),
                        relocalizeVision.size(), relocalizeWalls.describe(MIN_RELOCALIZE_SAMPLES));
            }
            telemetry.addData("Camera", camera.status());
            telemetry.addData("Pinpoint", "%s, %s", state, watchdog.status());
            telemetry.addData("IMU check", "%s, offset %.1f deg (must stay the same while turning)",
                    fallback.status(), fallback.offsetDeg());
            telemetry.addLine("HOLD: A shoot, RS button aim assist, Back vision mode (Start: odometry), B intakes reverse. "
                    + "Bumpers: intakes on/off. Y: indexer back 90. Dpad: trim");
            telemetry.update();

            Pose aimPose = shot == Shot.AIM || shot == Shot.FIRE ? aimer.aimPose() : null;
            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            Drawing.drawTags(packet.fieldOverlay(), pose, frame);
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

    /**
     * The pose against the audience wall (X > 0) or the far wall, whichever half John is in: camera
     * wall flush on it, shooter facing the field (audience: heading +90; far: -90); Y kept.
     */
    private static Pose backWallPose(Pose pose) {
        boolean audience = pose.x() >= 0;
        double x = (72.0 - RobotGeometry.WIDTH_IN / 2.0) * (audience ? 1.0 : -1.0);
        return new Pose(x, pose.y(), Math.toRadians(audience ? 90.0 : -90.0));
    }

    /** Start pose for the alliance with the Game Setup tile (or C1): X and heading are the wall's. */
    private Pose wallPose(AutoSettings settings) {
        return AutoSettings.startPose(red ? AutoSettings.Alliance.RED : AutoSettings.Alliance.BLUE, settings.tile());
    }

    // =========================================================================
    // Aim and shoot
    // =========================================================================

    /**
     * If nobody knows which CELL is up (no camera confirmation in UP_CELL_TRUST_S, nor the
     * autonomous' at the start), target the closest CELL of our HIVE. Returns what it did, or null.
     */
    private String pickCellIfUnknown(boolean upFromAuto, double seconds) {
        boolean known = tracker.msSinceConfirmed() <= UP_CELL_TRUST_S * 1000.0
                || (upFromAuto && seconds <= UP_CELL_TRUST_S);
        if (known) {
            return null;
        }
        Pose here = follower.pose();
        double y = HiveTags.hiveY(red);
        double toAudience = Math.hypot(HiveTags.cellX(true, true) - here.x(), y - here.y());
        double toFar = Math.hypot(HiveTags.cellX(false, true) - here.x(), y - here.y());
        tracker.setUpAudience(toAudience <= toFar);
        return "up CELL not known recently: the closest, the " + tracker.upCellName() + " CELL";
    }

    /** Aim and shoot from here if in the shooting zone, else drive into it first. */
    private String requestShot() {
        Pose here = follower.pose();
        double[] t = tracker.target();
        String problem = ShootingZone.problem(here.x(), here.y(), t[0], t[1], tracker.upAudience(),
                ZONE_TOLERANCE_IN, ZONE_TOLERANCE_DEG);
        if (problem == null) {
            return startAim();
        }
        // The whole field: the own-half rule is for the autonomous (r0169: from just over the middle
        // line it drove 37" instead of backing up 6")
        Pose spot = ShootingZone.nearestSpot(here, t[0], t[1], tracker.upAudience(), red, false);
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
        return MOVE_SPEED_IPS > 0 ? p.with(foresight.maxVelocityConstraint.at(MOVE_SPEED_IPS)) : p;
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

    private static double median(List<Double> v) {
        List<Double> s = new ArrayList<>(v);
        Collections.sort(s);
        int n = s.size();
        return n == 0 ? Double.NaN : n % 2 == 1 ? s.get(n / 2) : (s.get(n / 2 - 1) + s.get(n / 2)) / 2.0;
    }

    /** Median of each of x, y and heading (the heading around the first one, wrap-safe). */
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
