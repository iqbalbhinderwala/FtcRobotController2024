package org.firstinspires.ftc.teamcode.Barry.Auto;

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
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.Barry.Field.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.Barry.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Barry.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Barry.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Barry.Pedro.FtcToPedro;
import org.firstinspires.ftc.teamcode.Barry.Util.CsvLog;
import org.firstinspires.ftc.teamcode.Barry.Vision.CameraMount;
import org.firstinspires.ftc.teamcode.Barry.Vision.ClusterDetection;
import org.firstinspires.ftc.teamcode.Barry.Vision.LimelightAprilTagClusterDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * BIOBUZZ Auto. All positions are written for RED in the Pedro frame; BLUE is the same plan
 * reflected through the field center (x -> 144 - x, y -> 144 - y, heading + 180, tag id + 8).
 *
 * 1. INIT (robot still): pick the alliance (gamepad1 B = red, X = blue) and detect the start tile
 *    from where the audience-side cluster (red 34-37) appears: about straight ahead = C1, off to
 *    the side or not seen = B1 (dpad right / left forces C1 / B1, dpad up = detect again).
 *    Right side flush on the audience wall, camera/shooter out the left.
 * 2. C1: step STEP_OFF_WALL off the wall. B1: turn up to SEARCH_TURN_DEG clockwise while sliding
 *    diagonally off the wall, until the cluster is seen.
 * 3. Aim (turn in place) at the cluster center and shoot 4.
 * 4. Back flush on the audience wall, drive to the A1 corner with the back intake on (GARDEN).
 * 5. Move off the walls to LOOK_X / LOOK_Y and look at the HIVE: if the audience CELL is still up
 *    (cluster seen, high), shoot again from the C1 wall spot; otherwise the far CELL is up and we
 *    shoot from the far side (around C6). Turns only happen away from the walls.
 * 6. PARK in the LOADING ZONE. If PARK_DEADLINE_S is reached, anything still running is dropped
 *    and the robot goes to park.
 *
 * Shooter and intake are stubs for now (see Barry/Hardware). Camera mount: see CameraMount.
 *
 * Every run is logged to /sdcard/FIRST/barry_logs/hive_auto_<time>.csv (see CsvLog): one "init" or
 * "auto" row per loop (pose, step, both of our clusters as seen by the camera) and an "event" row for
 * each decision.
 */
@Config
@Autonomous(name = "[Barry] Hive Auto", group = "Barry")
public class HiveAuto extends LinearOpMode {

    public enum Alliance { RED, BLUE }

    public enum StartTile { C1, B1 }

    // ---- Robot outline (inches); the right side starts flush on the audience wall ----
    public static double ROBOT_HALF_WIDTH = 9.0;
    public static double ROBOT_HALF_LENGTH = 8.5;

    // ---- Red positions, Pedro frame (inches / degrees) ----
    public static double C1_X = 60.0;
    public static double B1_X = 36.0;
    /** Gap kept from the wall when driving along it. */
    public static double WALL_GAP = 0.5;
    /** C1: step this far off the audience wall before aiming, so the robot can turn without scraping. */
    public static double STEP_OFF_WALL = 1.0;
    /** B1 search: diagonal move (toward the midline, off the wall) while turning clockwise. */
    public static double SEARCH_DX = 5.0;
    public static double SEARCH_DY = 5.0;
    public static double SEARCH_TURN_DEG = 30.0;
    /** Spot to look at the HIVE from, far enough off both walls to turn in place. */
    public static double LOOK_X = 26.0;
    public static double LOOK_Y = 22.0;
    /** Far-side shooting spot when the far CELL is up (camera looks back toward the audience). */
    public static double FAR_SHOT_X = 63.0;
    public static double FAR_SHOT_Y = 130.0;
    /** Clear lane along column B used to go around the HIVE frame. */
    public static double LANE_X = 30.0;
    /** Park pose: partly inside the red LOADING ZONE (x 0-11, y 96-120, measure on the field). */
    public static double PARK_X = 12.0;
    public static double PARK_Y = 108.0;
    public static double PARK_HEADING_DEG = 180.0;

    // ---- Decisions and timing ----
    /** Cluster within this many degrees of the camera axis during INIT means we are on C1. */
    public static double START_C1_MAX_BEARING_DEG = 12.0;
    public static double AIM_TOLERANCE_DEG = 2.0;
    public static double AIM_TIMEOUT_MS = 1500;
    /** Time to look at the HIVE before deciding which CELL is up. */
    public static double LOOK_MS = 600;
    /**
     * The audience cluster counts as "CELL up" when seen above this height (inches). Up CELL tags are
     * about 49.9 in high, down about 35.6 in; the pivot is in between. Lower it for bench tests with
     * a cluster near the floor.
     */
    public static double CELL_UP_MIN_Z_IN = HiveTagGeometry.PIVOT_Z;
    /** Start parking at this match time no matter what (AUTO is 30 s, PARK is judged at its end). */
    public static double PARK_DEADLINE_S = 25.0;

    // Red cluster first tag IDs (blue = +8)
    private static final int AUDIENCE_CLUSTER = 34;
    private static final int FAR_CLUSTER = 30;

    private Follower follower;
    private Limelight3A limelight;
    private final LimelightAprilTagClusterDetector detector = new LimelightAprilTagClusterDetector();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    private Alliance alliance = Alliance.RED;
    private StartTile startTile = StartTile.B1;
    /** Driver override of the detected start tile, null = use the camera. */
    private StartTile startOverride = null;
    private List<ClusterDetection> clusters = new ArrayList<>();
    private boolean audienceCellUp = true;
    private String step = "init";
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime opModeTimer = new ElapsedTime();
    private CsvLog log;

    private static final String LOG_HEADER = "t_s,match_s,phase,step,alliance,start,x,y,heading_deg,mode,"
            + "aud_tags,aud_bearing_deg,aud_fx,aud_fy,aud_fz,far_tags,far_bearing_deg,far_fx,far_fy,far_fz,"
            + "shooter,intake,event";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        log = new CsvLog("hive_auto", LOG_HEADER);
        opModeTimer.reset();

        follower = Constants.create(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // INIT: choose alliance, detect the start tile (robot does not move)
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) alliance = Alliance.RED;
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.dpad_right) startOverride = StartTile.C1;
            if (gamepad1.dpad_left) startOverride = StartTile.B1;
            if (gamepad1.dpad_up) startOverride = null;
            clusters = detector.detect(limelight.getLatestResult());
            ClusterDetection audience = cluster(AUDIENCE_CLUSTER);
            StartTile detected = (audience != null && Math.abs(audience.bearingDeg()) <= START_C1_MAX_BEARING_DEG)
                    ? StartTile.C1 : StartTile.B1;
            startTile = startOverride != null ? startOverride : detected;
            follower.setPose(startPose());

            telemetry.addData("Alliance (B red / X blue)", alliance);
            telemetry.addData("Start tile", "%s (%s; dpad right C1 / left B1 / up detect)",
                    startTile, startOverride != null ? "forced" : "detected");
            telemetry.addData("Audience cluster", audience == null ? "not seen"
                    : String.format("bearing %.1f deg, %d tags", audience.bearingDeg(), audience.tagCount));
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();
            drawRobot(dashboard);
            logRow("init", "");
        }
        if (isStopRequested()) {
            log.close();
            limelight.stop();
            return;
        }

        matchTimer.reset();
        follower.setPose(startPose());
        logEvent(String.format("start: %s %s at %s (%s)", alliance, startTile, fmt(follower.pose()),
                startOverride != null ? "forced" : "detected"));
        Scheduler.reset();
        Scheduler.schedule(sequential(
                race(mainRoutine(), waitUntil(() -> matchTimer.seconds() >= PARK_DEADLINE_S)),
                instant(() -> { shooter.stop(); intake.off(); logEvent("parking"); }),
                lazy(this::park)));

        while (opModeIsActive()) {
            clusters = detector.detect(limelight.getLatestResult());
            follower.update();
            Scheduler.execute();

            telemetry.addData("Time", "%.1f s", matchTimer.seconds());
            telemetry.addData("Step", step);
            telemetry.addData("Alliance / start", "%s / %s", alliance, startTile);
            telemetry.addData("Pose", "x:%.1f y:%.1f h:%.1f", follower.pose().x(), follower.pose().y(),
                    Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Shooter", shooter.status());
            telemetry.addData("Intake", intake.status());
            telemetry.addData("Audience CELL up", audienceCellUp);
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();
            drawRobot(dashboard);
            logRow("auto", "");
        }

        logEvent("stop: " + fmt(follower.pose()));
        log.close();
        limelight.stop();
    }

    // =========================================================================
    // Routine
    // =========================================================================

    private Command mainRoutine() {
        return sequential(
                lazy(this::leaveWall),
                label("aim 1"), aimAtCluster(AUDIENCE_CLUSTER),
                label("shoot 1"), shooter.shoot(4),
                label("garden"), lazy(this::wallToGarden),
                label("look at HIVE"), lazy(this::lookAtHive),
                label("shot 2"), lazy(this::secondShot));
    }

    /**
     * C1: step straight off the wall. B1: turn clockwise while sliding off the wall until the cluster
     * is seen, then stop there. Either way the robot ends clear of the wall and can turn to aim.
     */
    private Command leaveWall() {
        Pose start = follower.pose();
        if (startTile == StartTile.C1) {
            Pose off = field(C1_X, ROBOT_HALF_WIDTH + STEP_OFF_WALL, 0);
            return sequential(label("step off wall"), follow(follower, line(start, off).constant(start.heading())));
        }
        Pose end = field(B1_X + SEARCH_DX, ROBOT_HALF_WIDTH + SEARCH_DY, -SEARCH_TURN_DEG);
        return sequential(
                label("search B1"),
                race(follow(follower, line(start, end).linear(start, end)),
                        waitUntil(() -> cluster(AUDIENCE_CLUSTER) != null)),
                instant(() -> follower.hold(follower.pose())));
    }

    /** Turn in place so the camera / shooter points at a cluster (red first ID), seen or nominal. */
    private Command aimAtCluster(int redFirstId) {
        return lazy(() -> {
            Pose robot = follower.pose();
            double[] target = clusterFieldXY(redFirstId, robot);
            double heading = CameraMount.headingToLookAt(robot, target[0], target[1]);
            logEvent(String.format("aim at cluster %d (%s): target (%.1f; %.1f) heading %.1f deg from %s",
                    allianceId(redFirstId), cluster(redFirstId) != null ? "seen" : "nominal",
                    target[0], target[1], Math.toDegrees(heading), fmt(robot)));
            return sequential(
                    instant(() -> follower.hold(new Pose(robot.x(), robot.y(), heading))),
                    race(waitUntil(() -> Math.abs(CameraMount.normalize(heading - follower.pose().heading()))
                                    <= Math.toRadians(AIM_TOLERANCE_DEG)),
                            waitMs(AIM_TIMEOUT_MS)));
        });
    }

    /** Back flush on the audience wall at heading 0, then drive to A1 with the back intake on. */
    private Command wallToGarden() {
        Pose start = follower.pose();
        double startX = redX(start.x());
        Pose atWall = field(startX, ROBOT_HALF_WIDTH + WALL_GAP, 0);
        Pose corner = field(ROBOT_HALF_LENGTH + WALL_GAP, ROBOT_HALF_WIDTH + WALL_GAP, 0);
        return sequential(
                follow(follower, line(start, atWall).linear(start, atWall)),
                intake.start(Intake.Side.BACK),
                follow(follower, line(atWall, corner).constant(atWall.heading())),
                waitMs(Intake.SETTLE_MS),
                intake.stop());
    }

    /** Move off the walls, point the camera at our HIVE center and decide which CELL is up. */
    private Command lookAtHive() {
        Pose start = follower.pose();
        double[] hive = toField(FtcToPedro.point(0.0, -HiveTagGeometry.HIVE_CENTER_TO_CENTER / 2.0));
        Pose spot = field(LOOK_X, LOOK_Y, 0);
        Pose look = spot.withHeading(CameraMount.headingToLookAt(spot, hive[0], hive[1]));
        return sequential(
                follow(follower, line(start, look).linear(start, look)),
                instant(() -> follower.hold(look)),
                waitMs(LOOK_MS),
                instant(() -> {
                    // From the audience side only the audience cluster faces us, and only while its
                    // CELL is up. Not seen (or low) means the HIVE tipped and the far CELL is up.
                    ClusterDetection audience = cluster(AUDIENCE_CLUSTER);
                    double z = audience == null ? Double.NaN
                            : CameraMount.toField(audience.x, audience.y, audience.z, follower.pose())[2];
                    audienceCellUp = audience != null && z > CELL_UP_MIN_Z_IN;
                    logEvent(String.format("HIVE check: audience cluster %s, z %.1f in -> %s CELL up",
                            audience == null ? "not seen" : "seen", z, audienceCellUp ? "audience" : "far"));
                }));
    }

    /** Drive to the shooting spot for whichever CELL is up, aim, and shoot 4. */
    private Command secondShot() {
        Pose start = follower.pose();
        if (audienceCellUp) {
            Pose spot = field(C1_X, ROBOT_HALF_WIDTH + WALL_GAP, 0);
            return sequential(
                    follow(follower, line(start, spot).linear(start, spot)),
                    aimAtCluster(AUDIENCE_CLUSTER),
                    shooter.shoot(4));
        }
        // Far CELL up: go up the lane on column B, then to the far side facing the HIVE
        Pose laneLow = field(LANE_X, 30.0, 0);
        Pose laneHigh = field(LANE_X, 115.0, 0);
        Pose spot = field(FAR_SHOT_X, FAR_SHOT_Y, 180);
        Path route = path(
                line(start, laneLow).constant(start.heading()),
                line(laneLow, laneHigh).constant(start.heading()),
                line(laneHigh, spot).linear(laneHigh.withHeading(start.heading()), spot));
        return sequential(
                follow(follower, route),
                aimAtCluster(FAR_CLUSTER),
                shooter.shoot(4));
    }

    /** Park partly inside the LOADING ZONE, going around the HIVE frame along column B. */
    private Command park() {
        step = "park";
        Pose start = follower.pose();
        double laneY = Math.max(30.0, Math.min(115.0, redY(start.y())));
        Pose lane = field(LANE_X, laneY, 0);
        Pose laneEnd = field(LANE_X, PARK_Y, 0);
        Pose parkPose = field(PARK_X, PARK_Y, PARK_HEADING_DEG);
        Path route = path(
                line(start, lane).constant(start.heading()),
                line(lane, laneEnd).constant(start.heading()),
                line(laneEnd, parkPose).linear(laneEnd.withHeading(start.heading()), parkPose));
        return follow(follower, route);
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private Command label(String name) {
        return instant(() -> {
            step = name;
            logEvent("step " + name);
        });
    }

    private int allianceId(int redFirstId) {
        return alliance == Alliance.BLUE ? redFirstId + 8 : redFirstId;
    }

    /** One log row: pose, step and both of our clusters as currently seen. */
    private void logRow(String phase, String event) {
        Pose pose = follower.pose();
        Object[] aud = clusterColumns(cluster(AUDIENCE_CLUSTER), pose);
        Object[] far = clusterColumns(cluster(FAR_CLUSTER), pose);
        log.row(opModeTimer.seconds(), phase.equals("init") ? null : matchTimer.seconds(), phase, step,
                alliance, startTile, pose.x(), pose.y(), Math.toDegrees(pose.heading()), follower.mode(),
                aud[0], aud[1], aud[2], aud[3], aud[4], far[0], far[1], far[2], far[3], far[4],
                shooter.status(), intake.status(), event);
    }

    private void logEvent(String event) {
        logRow("event", event);
    }

    /** tags, bearing, field x / y / z of a cluster, or blanks when it is not in view. */
    private static Object[] clusterColumns(ClusterDetection c, Pose robot) {
        if (c == null) {
            return new Object[]{null, null, null, null, null};
        }
        double[] f = CameraMount.toField(c.x, c.y, c.z, robot);
        return new Object[]{c.tagCount, c.bearingDeg(), f[0], f[1], f[2]};
    }

    private static String fmt(Pose p) {
        return String.format("(%.1f; %.1f; %.1f deg)", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private Pose startPose() {
        return field(startTile == StartTile.C1 ? C1_X : B1_X, ROBOT_HALF_WIDTH, 0);
    }

    /** Red pose (Pedro, degrees) -> the pose for the current alliance. */
    private Pose field(double redX, double redY, double redHeadingDeg) {
        double h = Math.toRadians(redHeadingDeg);
        if (alliance == Alliance.BLUE) {
            return new Pose(144.0 - redX, 144.0 - redY, CameraMount.normalize(h + Math.PI));
        }
        return new Pose(redX, redY, h);
    }

    /** Red point (Pedro) -> the point for the current alliance. */
    private double[] toField(double[] redXY) {
        return alliance == Alliance.BLUE
                ? new double[]{144.0 - redXY[0], 144.0 - redXY[1]}
                : redXY;
    }

    /** Current-alliance x / y back to red coordinates (the reflection is its own inverse). */
    private double redX(double x) {
        return alliance == Alliance.BLUE ? 144.0 - x : x;
    }

    private double redY(double y) {
        return alliance == Alliance.BLUE ? 144.0 - y : y;
    }

    /** The detection of a cluster given by its red first tag ID, or null if not in view. */
    private ClusterDetection cluster(int redFirstId) {
        return detector.find(clusters, alliance == Alliance.BLUE ? redFirstId + 8 : redFirstId);
    }

    /** Field position of a cluster: from the camera if seen, otherwise its nominal spot with its CELL up. */
    private double[] clusterFieldXY(int redFirstId, Pose robot) {
        ClusterDetection seen = cluster(redFirstId);
        if (seen != null) {
            double[] p = CameraMount.toField(seen.x, seen.y, seen.z, robot);
            return new double[]{p[0], p[1]};
        }
        boolean audienceEnd = redFirstId == AUDIENCE_CLUSTER;
        HiveTagGeometry.TagPose nominal = HiveTagGeometry.clusterPose(true, audienceEnd, audienceEnd ? 30 : -30);
        return toField(FtcToPedro.point(nominal.x, nominal.y));
    }

    private void drawRobot(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
        Drawing.drawRobot(packet.fieldOverlay(), follower.pose());
        dashboard.sendTelemetryPacket(packet);
    }
}
