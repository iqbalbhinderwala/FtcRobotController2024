package org.firstinspires.ftc.teamcode.John.Auto;

import org.firstinspires.ftc.teamcode.John.FieldGeometry;
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
import org.firstinspires.ftc.teamcode.John.Hardware.Flywheel;
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
 * he is already partly in the LOADING ZONE (that scores), otherwise he tries a spot PARK_ALT_OFFSET_IN
 * shallower (never deeper). He parks on the seam of the zone's end nearer his route, never driving
 * through the zone.
 *
 * Slow stretches (the GARDEN pass, the last park stretch) are PathWatcher.creep: a capped-speed
 * Pedro path never got John moving (r0109 / r0111: its coast feedforward is below the static
 * friction at 12-15 in/s).
 * STEP_MODE (testing) waits for gamepad1 Y before each stage and turns the park deadline off.
 *
 * Every run is logged to /sdcard/FIRST/john_v2_logs/r<run>_john_auto_<time>.csv: one row per loop
 * and an event row per decision. The last pose and the up CELL go to the blackboard for
 * [John] TeleOp (preselected), which starts from them.
 */
@Config("JohnAuto")
@Autonomous(name = "[John] Auto", group = "John", preselectTeleOp = "[John] TeleOp")
public class JohnAuto extends LinearOpMode {

    // ---- Red positions, FTC field coordinates (inches); blue is reflected ----
    /** Audience wall and red wall. */
    public static double AUDIENCE_WALL_X = FieldGeometry.WALL;
    public static double RED_WALL_Y = -FieldGeometry.WALL;
    /** Gap kept from a wall when driving along it. */
    public static double WALL_GAP = 0.5;
    /** Extra room between the turning robot's corner and the wall. */
    public static double TURN_MARGIN = 0.5;
    /** GARDEN: the 4 balls lie along the audience wall; the first (farthest from the corner) at this Y. */
    public static double GARDEN_FIRST_BALL_Y = -FieldGeometry.WALL + 10.1;   // 10.1" from the red wall (was -61.9 with the wall at 72)
    public static double BALL_RADIUS = 1.4;
    /** Start the slow stretch this far before intake A reaches the first ball. */
    public static double SLOW_BEFORE_BALL_IN = 2.0;   // 2 (was 4): r1149 / r1152 slowed down with room to spare
    /**
     * The fast run along the audience wall to the GARDEN: this far off the wall and no faster than
     * this (0 = no cap); the slow stretch through the balls then slides onto the wall (WALL_GAP).
     * 2026-10-25, was 0.5 in and uncapped: it ran ~54 in/s rubbing the wall.
     */
    public static double GARDEN_RUN_GAP_IN = 1.0;
    public static double GARDEN_RUN_SPEED_IPS = 30.0;
    /**
     * Where the slow collect ends: John's intake A end this far from the alliance wall in the GARDEN
     * corner; negative = into the wall, so he creeps until he touches it and stalls there ("as far as
     * it goes", PathWatcher). 2026-11-14 (r1083), with WALL_GAP 0.5: the odometry said 1.1" from the
     * wall, the real gap was 6-7 cm (2.4-2.8"); John is 43.2 cm (17.0") intake to intake, as in
     * RobotGeometry, so that was odometry error, which varies run to run (r1070 stopped 3.1" short).
     */
    public static double GARDEN_END_GAP_IN = -2.5;
    /**
     * GARDEN approach angle (2026-11-14): John runs along the audience wall at 90 + this (red frame),
     * intake A angled toward the wall, its camera-side corner (tapered in toward the intake) leading
     * along the wall, so the balls against the wall meet the intake face instead of slipping past its
     * side; through the slow stretch he turns back to 90 and arrives square at the red wall. The
     * center is placed so that leading corner stays GARDEN_RUN_GAP_IN / WALL_GAP off the wall.
     * 0 = the old square approach (r1085 picked up 3 of 4, maybe missing the last one).
     */
    /**
     * 0 (2026-11-15): at the practice field the 15 deg corner stalled against the wall; square it is.
     */
    public static double GARDEN_ANGLE_DEG = 0.0;
    /**
     * Before the balls, still square (90), John strafes gently into the audience wall until he stops
     * (GARDEN_TOUCH_*): the difference between where he is (wall - half his width) and where the
     * odometry says he is, up to GARDEN_TOUCH_MAX_FIX_IN, shifts the GARDEN's own targets (the
     * angled run line and the end), so the leading corner runs the planned gap from the real wall.
     * The odometry itself is not changed. 2026-11-14: the real gap varied by 1-2".
     */
    public static boolean GARDEN_TOUCH = true;   // on since 2026-11-15: all six day-2 matches ran it (TEST square)
    /**
     * The touch drives diagonally: forward toward the balls (GARDEN_TOUCH_FORWARD) and sideways into
     * the wall (GARDEN_TOUCH_POWER). On mecanum a diagonal grips far better than a pure strafe (John v2
     * barely strafes at 0.3, and turns when strafing). It starts GARDEN_TOUCH_LEAD_IN before the slow
     * stretch and is over once he has stopped moving sideways (against the wall); reaching the slow
     * stretch first: not used. It only counts if he moved at least GARDEN_TOUCH_MIN_MOVE_IN toward the
     * wall first (he starts GARDEN_RUN_GAP_IN off it).
     */
    public static double GARDEN_TOUCH_POWER = 0.35;   // to break free sideways
    /**
     * A gentle tap: once moving sideways, the sideways power holds about this speed (2026-11-15 practice
     * field: at a constant 0.35 John met the wall at 5-5.7 in/s, shaking the balls along it).
     */
    public static double GARDEN_TOUCH_SIDE_IPS = 2.5;
    public static double GARDEN_TOUCH_MIN_POWER = 0.18;
    public static double GARDEN_TOUCH_FORWARD = 0.20;
    public static double GARDEN_TOUCH_LEAD_IN = 12.0;
    public static double GARDEN_TOUCH_MIN_MOVE_IN = 0.3;
    /**
     * After a usable touch the angled run's leading corner runs this close to the wall instead of
     * GARDEN_RUN_GAP_IN (that margin was for odometry error, which the touch measured): with 1.0 the
     * corner moved 1" back off the wall right after touching it.
     */
    /**
     * +0.25: just off the wall (the tap measured it to ~0.3"). Riding it at -0.25 (into the wall) rubbed:
     * the creep slowed to 2-3 in/s and John caught the wall crease at the B/C tile seam (match 1, r1164).
     */
    public static double GARDEN_TOUCHED_GAP_IN = 0.25;
    public static double GARDEN_TOUCH_MAX_S = 1.5;
    public static double GARDEN_TOUCH_MAX_FIX_IN = 2.5;
    /** The touch is done once slower than this SIDEWAYS for GARDEN_TOUCH_STILL_MS (after GARDEN_TOUCH_GRACE_MS). */
    public static double GARDEN_TOUCH_STILL_IPS = 0.8;
    public static double GARDEN_TOUCH_STILL_MS = 250;
    public static double GARDEN_TOUCH_GRACE_MS = 250;
    /** Stalled this close to the collect's end (which lies past the red wall): back off and push again. */
    public static double GARDEN_NEAR_END_IN = 7.0;
    public static int GARDEN_END_RETRIES = 0;   // the back-off cost 2 s and gained nothing (r1149): off
    /**
     * The turn back to 90 is complete this far before the collect's end (which lies past the red wall).
     * 8 (was 4): r1135 reached the last two balls, stacked along the wall into the corner, still at
     * 98 deg; the corner pinned them instead of the intake face taking them.
     */
    public static double GARDEN_TURN_DONE_BEFORE_IN = 8.0;
    /**
     * Shooter.PRESPIN_RAMP_RPM_PER_S during Auto (volley 2 raised gradually, ball 2 waits less); put
     * back when Auto ends, so TeleOp keeps the instant raise. Run in all six day-2 matches.
     */
    public static double AUTO_PRESPIN_RAMP_RPM_PER_S = 150;
    /** Which settings this run uses ("competition", or a JohnAutoTest variant): in the start event. */
    public static String VARIANT = "competition (square, wall touch, volley-2 ramp)";
    /**
     * Park: partly inside the red LOADING ZONE (A5: X -48 to -24, Y -72 to -60.4), camera wall on the
     * wall, never driving through it (a partner may be in it). John comes in from the end nearer his
     * route: from the audience side (X > the zone's middle, e.g. from A2 or the look spot) he parks
     * centered on the A4 / A5 seam (X -24), from the far side (e.g. after shooting from B6) on the
     * A5 / A6 seam (X -48); PARK_INTO_ZONE_IN more into the zone (0: centered on the seam, 8.5" of
     * John inside, which counts).
     */
    public static double PARK_INTO_ZONE_IN = 0.0;
    /**
     * 0: the camera wall against the red wall, the shooter (and camera) facing the field / HIVE; the
     * last stretch slides along the wall along John's length (2026-10-24, was 90: an intake end to
     * the wall). Parked like this the camera sees the HIVE, which helps the TeleOp's start check.
     */
    public static double PARK_HEADING_DEG = 0.0;
    /**
     * The drive to the shot-2 spot is blocked (a partner parked on it: match 4 of 2026-11-15, r1173,
     * waited 1 s and pushed again for nothing) and John is in range of the CELL with this much
     * tolerance (ShootingZone.problem): he shoots from there at once.
     */
    public static double SHOT2_BLOCKED_TOLERANCE_IN = 2.0;
    public static double SHOT2_BLOCKED_TOLERANCE_DEG = 3.0;
    /**
     * Retries of the drive to the shot-2 spot after a block out of range: 0, he aims and shoots
     * from where he was stopped. A robot parked on the spot does not move; the two retries (1 s
     * wait and ~0.95 s pushing each) cost ~3.9 s of volley 2 in the match-4 simulation (partner on
     * the F2 / F1 seam).
     */
    public static int SHOT2_RETRIES = 0;
    /**
     * The drive to the park approach is blocked: drive at once to the LOADING ZONE's other end, no
     * retry of the same way (match 4, r1173: the far-end approach was in F1, where the partner had
     * parked; two retries into it and Auto ended unparked). A robot parked next to the zone can only
     * be at one end. The other end's approach gets the usual retries.
     */
    public static boolean PARK_OTHER_END_IF_BLOCKED = true;
    /**
     * Parking decides SWARM (LEAVE + PARK >= 16: both robots parked), so the park timer keeps this
     * much more than the plain estimate + PARK_MARGIN_S: room for one blocked try (~0.9 s) and the
     * way to the other end (2026-11-19; the match-4 simulations parked with 0.2-0.7 s to spare
     * without it).
     */
    public static double PARK_BLOCK_RESERVE_S = 1.5;
    /**
     * AUTO park end: a BLOCKED / PUSHED this close (red frame) to the nearer end's approach, nearer
     * it than the other end's, earlier in the Auto, means a robot there: park at the other end (by
     * the lane) from the start. Match 4: blocked 19.8" from the far end's approach.
     */
    public static double PARK_AVOID_TRIP_IN = 30.0;
    /**
     * The last resort: this long before the end, not in the LOADING ZONE, John drops whatever park
     * step runs and drives straight to the nearest spot PARK_DASH_OVERLAP_IN inside the zone, at
     * PARK_DASH_IPS.
     */
    public static double PARK_DASH_S = 1.5;
    public static double PARK_DASH_OVERLAP_IN = 3.0;
    public static double PARK_DASH_IPS = 30.0;
    /**
     * The way to the other end (red frame; a partner is parked somewhere at the first end, maybe
     * half in the zone, e.g. on the F2 / F1 seam): back off PARK_BACKOFF_IN straight out from the
     * wall (not back along his way: pinned at an angle, that kept a corner on a robot on the seam
     * in the simulation), then along a lane below the parking strip, turned to the park heading, to the other
     * end's park X, and only there into the zone. The lane: John's center PARK_LANE_OFF_ZONE_IN out
     * from the zone's line (an 18" robot against the wall sticks 6.4" out of the 11.6" zone, John's
     * half width 8.6", 2" between). Not straight across: from match 4's shot-2 spot that line runs
     * through a robot on the F2 / F1 seam (simulation, 2026-11-19).
     */
    public static double PARK_BACKOFF_IN = 3.0;
    public static double PARK_LANE_OFF_ZONE_IN = 17.0;
    /** The lane ends this far out from the park spot; from there into the zone (the shortcut below then creeps in). */
    public static double PARK_LANE_RISE_STOP_IN = 4.0;
    /**
     * On the way to a park approach (outside the LOADING ZONE), John already overlaps the zone by
     * this much both ways: he leaves the approach out and drives straight from there to the park
     * spot (the last stretch, at PARK_SPEED_IPS). The match-4 simulation (2026-11-19): going to the
     * other end, the drive crossed the zone, left it for the approach and came back in. If that last
     * stretch is blocked, he stays (already partly in, parkElsewhere).
     */
    public static double PARK_STOP_OVERLAP_IN = 4.0;
    /** If the park is blocked (and John is not partly in yet): a spot this much shallower (never deeper). */
    public static double PARK_ALT_OFFSET_IN = 5.0;
    /** Red LOADING ZONE (FTC); PARK scores when the robot is partly inside (by at least PARK_OVERLAP_IN). */
    public static double LOADING_ZONE_MIN_X = -48.0;
    public static double LOADING_ZONE_MAX_X = -24.0;
    public static double LOADING_ZONE_MAX_Y = -FieldGeometry.WALL + 11.6;   // 11.6" from the red wall (was -60.4 with the wall at 72)
    public static double PARK_OVERLAP_IN = 1.0;
    /**
     * Before the last park stretch: just off the red wall, this far outside the seam John parks on, at
     * the park heading; the drive there goes around the HIVE frame (ShootingZone.route). The last
     * stretch then slides along the wall onto the seam, so a robot in the way is pushed along the
     * wall, not into it.
     */
    public static double PARK_APPROACH_OUT_IN = 12.0;
    /**
     * Parked, the camera wall stays this far off the red wall: John must not touch the wall when
     * parked, with room for odometry error, and it clears a ball lying against the wall (2.8"; r0186
     * was stopped by one). The zone is 11.6" deep, so about 8.6" of John stays inside.
     */
    public static double PARK_WALL_GAP_IN = 3.0;
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
    // 300 / 1000 (were 600 / 2500, 2026-10-24): r0163 / r0164 / r0168 all waited the full 2.5 s
    // for tags that a tipped HIVE never shows; at ~30 frames / s, 1 s without a tag is clear enough.
    public static double LOOK_MIN_MS = 300;
    public static double LOOK_TIMEOUT_MS = 1000;
    public static double CONFIRM_MAX_AGE_MS = 200;
    /** Times per volley it may stop for a tip and go to the other side. */
    public static int MAX_SIDE_SWITCHES = 2;

    // ---- Speeds and timing ----
    /** 4 (2026-10-25; was 8, before that 12). */
    public static double COLLECT_SPEED_IPS = 4.0;
    /** After the GARDEN the intake runs this much longer (the balls move on toward the indexer). */
    public static double INTAKE_AFTER_GARDEN_S = 2.0;
    /**
     * From the start of aim 2 to the end of volley 2 the GARDEN-side intake (A) runs, standing still,
     * so the balls keep moving into the indexer (late balls caused the indexer jams, e.g. r0179). Off
     * while driving across the field: a loose ball picked up there would make 5 (G407).
     */
    public static boolean FEED_DURING_VOLLEY_2 = true;
    /**
     * Flywheel spin-up (1.45 s from rest) ahead of the volleys: at START for volley 1 (was at the aim,
     * so John waited about 1.2 s aimed: r0260-r0266), and on the drive to a shooting spot once the
     * estimated drive left is SPIN_LEAD_S (was at the end of the drive: about 1.0 s aimed). Not from
     * the start of that drive, so the full-power boost (8.5 A) does not add to the drive's hardest
     * acceleration. Holding speed takes about 2.3 A.
     * John v2 (gear belt): spin-up about 2.1 s; with 1.6 the volley 2 still waited 1.25 s aimed (r1061).
     */
    public static boolean SPIN_AT_START = true;
    public static double SPIN_LEAD_S = 2.8;
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
    /** John v2: 40 (was 30): shot 2 was estimated at 8.9 s and took ~7.1 s (r1076); r1084 skipped volley 2 by 0.1 s. */
    public static double AVG_DRIVE_IPS = 40.0;
    public static double DRIVE_OVERHEAD_S = 0.6;
    public static double PARK_OVERHEAD_S = 1.0;
    public static double AIM_S = 0.8;   // John v2: the aim takes ~0.7 s (was 1.2)
    public static double VOLLEY_S = 3.2;   // John v2: 5 steps in 3.0-3.4 s (was 3.5)
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
            + "target_src,up_cell,tags,watchdog,idx_deg,idx_target_deg,idx_amps,cam_x,cam_y,cam_err_in,"
            + "hive,up_height_in,fw_rpm,fw_target,fw_mode,fw_amps,fw_f,battery_v,event";

    private Follower follower;
    private ForesightConfig foresight;
    private OdometryWatchdog watchdog;
    private PathWatcher watcher;
    private HiveCamera camera;
    private CellTracker tracker;
    private Aimer aimer;
    private Shooter shooter;
    /** Driving to this shooting spot (field frame): spin the flywheel up SPIN_LEAD_S before it; null: not. */
    private Pose spinGoal;
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
    /** The GARDEN balls are on board: aim / volley 2 run the GARDEN-side intake to feed the indexer. */
    private boolean feedFromIntake;
    private CsvLog log;

    @Override
    public void runOpMode() {
        double ramp = Shooter.PRESPIN_RAMP_RPM_PER_S;
        try {
            Shooter.PRESPIN_RAMP_RPM_PER_S = AUTO_PRESPIN_RAMP_RPM_PER_S;
            runAuto();
        } finally {
            Shooter.PRESPIN_RAMP_RPM_PER_S = ramp;
        }
    }

    private void runAuto() {
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
        parkEnd = settings.parkEnd();
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
        logEvent("variant: " + VARIANT);
        logEvent(String.format(Locale.US, "start: %s %s at %s (%s), delay %.0f s", settings.alliance(), settings.tile(),
                Drawing.format(start), useCamera(startFix)
                        ? String.format(Locale.US, "camera Y, %d readings, tile %+.1f", startFix.count(),
                                startFix.y() - settings.startPose().y())
                        : "tile Y", settings.delayS()));

        Scheduler.reset();
        Scheduler.schedule(sequential(
                waitMs(settings.delayS() * 1000.0),
                race(mainRoutine(), waitUntil(() ->
                        AUTO_S - autoSeconds() <= parkSeconds(follower.pose()) + PARK_MARGIN_S + PARK_BLOCK_RESERVE_S)),
                instant(() -> {
                    if (!routineDone) {
                        logEvent(String.format(Locale.US, "time to park: %.1f s left, park about %.1f s from %s",
                                AUTO_S - autoSeconds(), parkSeconds(follower.pose()), Drawing.format(follower.pose())));
                    }
                    shooter.cancel();
                    shooter.setSpin(false);
                    spinGoal = null;
                    intake.off();
                }),
                lazy(() -> routineDone ? label("park")
                        : instant(() -> step = "park")),   // forced by the time: no Y
                race(limit("park", PARK_MAX_S, lazy(this::park)),
                        waitUntil(() -> AUTO_S - autoSeconds() <= PARK_DASH_S && !inLoadingZone(red(follower.pose())))),
                lazy(() -> inLoadingZone(red(follower.pose())) ? instant(() -> { }) : parkDash()),
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
            if (spinGoal != null && driveSeconds(pose, spinGoal) <= SPIN_LEAD_S) {
                spinGoal = null;
                shooter.setSpin(true);
                logEvent(String.format(Locale.US, "flywheel on: about %.1f s of driving left to the shooting spot",
                        SPIN_LEAD_S));
            }
            double[] target = tracker.target();
            shooter.setDistance(ShootingZone.shotDistance(pose.x(), pose.y(), tracker.upAudience(), !blue));
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
                instant(() -> {
                    if (SPIN_AT_START) {
                        shooter.setSpin(true);
                        logEvent("flywheel on at the start (for volley 1)");
                    }
                }),
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
                                    ifInRange("volley 2", () -> sequential(
                                            aim(),
                                            volley("volley 2", Shooter.INDEXER_STEPS_PER_VOLLEY, 0)))))));
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
                    spinGoal = null;
                    aimResult = Aimer.Result.AIMING;
                    logEvent(aimer.start());
                    if (feedFromIntake) {
                        // Standing still from here to the end of the volley: feed the indexer
                        intakeOffAtS = Double.NaN;
                        intake.only(Intake.Side.A, Intake.POWER);
                        logEvent("intake A on: feeding the indexer for the volley");
                    }
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
                    if (!feedFromIntake && !intake.status().equals("off")) {
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
                    if (!intake.status().equals("off")) {
                        intake.off();   // before any drive (a side switch, the park)
                        logEvent(name + ": intake off");
                    }
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
                    tracker.startTip(!wasAudience[0]);   // TIPPING: the camera may not undo this mid-swing
                    logEvent(name + ": going to the other side, the " + tracker.upCellName() + " CELL");
                    return sequential(
                            label(name + ": to the other side"),
                            guard(name + ": the other side", this::shotSeconds, this::shotEnd, () -> sequential(
                                    lazy(this::toShootingSpot),
                                    ifInRange(name, () -> sequential(
                                            aim(),
                                            volley(name, Math.max(1, left[0]), switches + 1))))));
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
        double heading = Math.toRadians(90.0);
        double runHeading = Math.toRadians(90.0 + GARDEN_ANGLE_DEG);
        double wallX = AUDIENCE_WALL_X - extentX(heading) - WALL_GAP;
        // Square along the wall up to the touch; angled from there (the leading, camera-side corner of
        // intake A nearest the wall)
        double squareRunX = AUDIENCE_WALL_X - extentX(heading) - GARDEN_RUN_GAP_IN;
        double runX = AUDIENCE_WALL_X - extentX(runHeading) - GARDEN_RUN_GAP_IN;
        // Turn in place at 90, then out to the wall line
        Pose turned = here.withHeading(heading);
        Pose atWall = new Pose(squareRunX, here.y(), heading);
        // Intake A is at the robot's -X end, which points toward the red wall (-Y) at heading 90
        double slowY = gardenSlowY(runHeading, runX);
        double endY = RED_WALL_Y + halfLength + GARDEN_END_GAP_IN;
        // The wall tap right where John reaches the wall line (not GARDEN_TOUCH_LEAD_IN before the balls:
        // the tap drives forward slowly, and r1149 / r1152 crept ~15" at 5 in/s); then the fast run
        Pose touchAt = new Pose(squareRunX, here.y(), heading);
        gardenFixX = 0.0;
        gardenTouched = false;
        // The angled run line: from the wall touch when it worked (smaller margin), else the planned one
        // gardenFixX = odometry - real: a real position X is odometry X + gardenFixX (2026-11-15 practice
        // field: subtracting it moved John off the wall by twice the error, 0.9-2.1")
        DoubleSupplier angledX = () -> gardenTouched
                ? AUDIENCE_WALL_X - extentX(runHeading) - GARDEN_TOUCHED_GAP_IN + gardenFixX : runX;
        DoubleSupplier squareX = () -> gardenTouched
                ? AUDIENCE_WALL_X - extentX(heading) - GARDEN_TOUCHED_GAP_IN + gardenFixX : wallX;
        logEvent(String.format(Locale.US, "garden: wall at x %.1f (run x %.1f at %.0f deg), slow from y %.1f to %.1f "
                        + "at %.0f in/s, turning back to 90%s", wallX, runX, 90.0 + GARDEN_ANGLE_DEG, slowY, endY,
                COLLECT_SPEED_IPS, GARDEN_TOUCH ? "; wall touch first" : ""));
        return sequential(
                instant(() -> follower.hold(field(turned))),
                race(waitUntil(() -> Math.abs(normalize(field(turned).heading() - follower.pose().heading()))
                                <= Math.toRadians(TURN_TOLERANCE_DEG)),
                        waitMs(TURN_TIMEOUT_MS)),
                watcher.follow("garden: to the wall", () -> around(field(atWall), 0)),
                intake.start(Intake.Side.A),
                GARDEN_TOUCH ? touchWall(wallX + WALL_GAP, slowY + 6.0) : instant(() -> { }),
                // The fast run along the wall (on the line the tap corrected), to the slow point
                watcher.follow("garden: along the wall", () -> toward(field(new Pose(gardenTouched
                        ? AUDIENCE_WALL_X - extentX(heading) - GARDEN_RUN_GAP_IN + gardenFixX : squareRunX,   // 1" off: clear of the creases
                        Math.min(follower.pose() == null ? here.y() : red(follower.pose()).y(), slowY), heading)), true,
                        GARDEN_RUN_SPEED_IPS)),
                // Angle onto the run line (shifted by the touch), then creep through the balls
                instant(() -> follower.hold(field(new Pose(angledX.getAsDouble(), slowY, runHeading)))),
                race(waitUntil(() -> {
                            Pose g = field(new Pose(angledX.getAsDouble(), slowY, runHeading));
                            Pose p = follower.pose();
                            return Math.hypot(p.x() - g.x(), p.y() - g.y()) <= 0.75
                                    && Math.abs(normalize(g.heading() - p.heading())) <= Math.toRadians(TURN_TOLERANCE_DEG);
                        }),
                        waitMs(TURN_TIMEOUT_MS)),
                // Turning back to 90 and onto the square wall line by the same point, so the leading
                // corner keeps its gap (one straight line drifted it 1" off mid-way), then straight on
                // A stall anywhere in it (on the first balls): back off, push once more, then on to the next part
                watcher.creep("garden: turn back", () -> field(new Pose(squareX.getAsDouble(),
                        endY + GARDEN_TURN_DONE_BEFORE_IN, heading)), COLLECT_SPEED_IPS, 0.0, 1000.0, GARDEN_END_RETRIES),
                watcher.creep("garden: collect", () -> field(new Pose(squareX.getAsDouble(), endY, heading)),
                        COLLECT_SPEED_IPS, Double.NaN, GARDEN_NEAR_END_IN, GARDEN_END_RETRIES),
                // The intake keeps running through the next stages, so the balls work their way to
                // the indexer while driving; the next volley (or the park) stops it
                instant(() -> {
                    intakeOffAtS = autoSeconds() + INTAKE_AFTER_GARDEN_S;
                    feedFromIntake = FEED_DURING_VOLLEY_2;
                    logEvent(String.format(Locale.US, "garden done: the intake keeps running %.1f s more", INTAKE_AFTER_GARDEN_S));
                }));
    }

    /**
     * {@code shoot} only if John is in range of the up CELL now (ShootingZone.problem with the
     * SHOT2_BLOCKED_TOLERANCE_* tolerance); else he keeps the balls (for the TeleOp) and the routine
     * goes on, i.e. to the park. A block on the way to the spot, or no spot on the field, leaves him
     * out of range: a ball from there would likely miss (2026-11-19: 40.8" from the CELL, under the
     * speed table's 42", in the match-4 simulation).
     */
    private Command ifInRange(String name, java.util.function.Supplier<Command> shoot) {
        return lazy(() -> {
            Pose p = follower.pose();
            double[] c = tracker.target();
            String problem = ShootingZone.problem(p.x(), p.y(), c[0], c[1], tracker.upAudience(),
                    SHOT2_BLOCKED_TOLERANCE_IN, SHOT2_BLOCKED_TOLERANCE_DEG);
            if (problem == null) {
                return shoot.get();
            }
            return instant(() -> {
                follower.hold(follower.pose());
                shooter.setSpin(false);
                spinGoal = null;
                logEvent(name + ": not in range at " + Drawing.format(p) + " (" + problem + "): keeping the balls");
            });
        });
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
        return sequential(
                instant(() -> spinGoal = goal),   // the main loop spins up SPIN_LEAD_S before the end
                watcher.follow("to shot 2", () -> aimedRoute(goal), SHOT2_RETRIES, () -> {
                    Pose p = follower.pose();
                    double[] c = tracker.target();
                    return ShootingZone.problem(p.x(), p.y(), c[0], c[1], tracker.upAudience(),
                            SHOT2_BLOCKED_TOLERANCE_IN, SHOT2_BLOCKED_TOLERANCE_DEG) == null;
                }));
    }

    /**
     * Around the HIVE frame to the LOADING ZONE; the last stretch slow. The way to the nearer end
     * blocked: the other end at once (PARK_OTHER_END_IF_BLOCKED). The last stretch blocked: stay when
     * already partly in the LOADING ZONE, else try the other spots along it.
     */
    private Command park() {
        Pose here = red(follower.pose());
        boolean nearer = fromAudienceSide(here);
        boolean fromAudience = parkFromAudience(here);
        logEvent("park end: " + (fromAudience ? "audience" : "far") + " (" + parkEndWhy + ")");
        // Not the nearer end: something is (or the partner parks) at the nearer one; the lane keeps
        // clear of the parking strip there
        return fromAudience == nearer ? parkAt(fromAudience, PARK_OTHER_END_IF_BLOCKED) : parkOtherEnd(fromAudience);
    }

    private AutoSettings.ParkEnd parkEnd = AutoSettings.ParkEnd.AUTO;
    private String parkEndWhy = "";

    /** The end to park at (red frame pose): Game Setup's, else the nearer one unless a block was near it. */
    private boolean parkFromAudience(Pose red) {
        if (parkEnd != AutoSettings.ParkEnd.AUTO) {
            parkEndWhy = "Game Setup";
            return parkEnd == AutoSettings.ParkEnd.AUDIENCE;
        }
        boolean nearer = fromAudienceSide(red);
        Pose trip = watcher.lastTripPose();
        if (trip != null) {
            Pose t = red(trip);
            double dNear = distance(t, parkApproach(nearer)), dOther = distance(t, parkApproach(!nearer));
            if (dNear < PARK_AVOID_TRIP_IN && dNear < dOther) {
                parkEndWhy = String.format(Locale.US, "away from the block at %s, %.0f in from the %s end",
                        Drawing.format(trip), dNear, nearer ? "audience" : "far");
                return !nearer;
            }
        }
        parkEndWhy = "the nearer end";
        return nearer;
    }

    /** The park approach of one end (red frame). */
    private static Pose parkApproach(boolean fromAudience) {
        double side = fromAudience ? 1.0 : -1.0;
        return new Pose(parkX(fromAudience) + side * PARK_APPROACH_OUT_IN, parkY() + PARK_APPROACH_OFF_WALL_IN,
                Math.toRadians(PARK_HEADING_DEG));
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    /**
     * The last resort (PARK_DASH_S before the end, not in the LOADING ZONE): straight to the
     * nearest spot PARK_DASH_OVERLAP_IN inside it, keeping the heading.
     */
    private Command parkDash() {
        Pose red = red(follower.pose());
        double c = Math.abs(Math.cos(red.heading())), s = Math.abs(Math.sin(red.heading()));
        double halfX = RobotGeometry.LENGTH_IN / 2.0 * c + RobotGeometry.WIDTH_IN / 2.0 * s;
        double halfY = RobotGeometry.LENGTH_IN / 2.0 * s + RobotGeometry.WIDTH_IN / 2.0 * c;
        double x = Math.max(LOADING_ZONE_MIN_X + PARK_DASH_OVERLAP_IN - halfX,
                Math.min(LOADING_ZONE_MAX_X - PARK_DASH_OVERLAP_IN + halfX, red.x()));
        double y = Math.max(RED_WALL_Y + halfY + 0.5, Math.min(LOADING_ZONE_MAX_Y - PARK_DASH_OVERLAP_IN + halfY, red.y()));
        Pose spot = new Pose(x, y, red.heading());
        logEvent(String.format(Locale.US, "park: %.1f s left, NOT in the LOADING ZONE: straight to (%.1f, %.1f)",
                AUTO_S - autoSeconds(), field(spot).x(), field(spot).y()));
        return sequential(
                watcher.creep("park dash", () -> field(spot), PARK_DASH_IPS),
                instant(() -> logEvent((inLoadingZone(red(follower.pose())) ? "parked (dash) at " : "park dash ended NOT in the LOADING ZONE at ")
                        + Drawing.format(follower.pose()))));
    }

    /** Park at one end of the LOADING ZONE; {@code otherEndIfBlocked}: no retries on the way there. */
    private Command parkAt(boolean fromAudience, boolean otherEndIfBlocked) {
        double heading = Math.toRadians(PARK_HEADING_DEG);
        double side = fromAudience ? 1.0 : -1.0;   // +X: the A4 side, -X: the A6 side
        double parkX = parkX(fromAudience);
        Pose approach = new Pose(parkX + side * PARK_APPROACH_OUT_IN, parkY() + PARK_APPROACH_OFF_WALL_IN, heading);
        Pose parkPose = new Pose(parkX, parkY(), heading);
        logEvent(String.format(Locale.US, "park on the %s seam at (%.1f, %.1f), coming along the wall from the %s side, from %s",
                fromAudience ? "A4 / A5" : "A5 / A6", parkX, parkY(), fromAudience ? "audience" : "far",
                Drawing.format(follower.pose())));
        return sequential(
                watcher.follow("park: around the HIVE", () -> around(field(approach), 0),
                        otherEndIfBlocked ? 0 : PathWatcher.RETRIES, null,
                        () -> inLoadingZone(red(follower.pose()), PARK_STOP_OVERLAP_IN)),
                lazy(() -> !watcher.gaveUp() && inLoadingZone(red(follower.pose()), PARK_STOP_OVERLAP_IN)
                        ? sequential(
                                instant(() -> logEvent("park: in the LOADING ZONE on the way at " + Drawing.format(follower.pose())
                                        + ": straight to the park spot")),
                                parkFinish(parkX, side, parkPose))
                        : watcher.gaveUp() && otherEndIfBlocked && !inLoadingZone(red(follower.pose()))
                        ? sequential(
                                instant(() -> logEvent("park: the way to the " + (fromAudience ? "audience" : "far")
                                        + " end is blocked: parking at the other end")),
                                parkOtherEnd(!fromAudience))
                        : parkFinish(parkX, side, parkPose)));
    }

    /**
     * After the way to one end was blocked: back off out from the wall,
     * along the lane below the parking strip to the other end's park X, up into the zone; once 4"
     * in (PARK_STOP_OVERLAP_IN), the last stretch straight to the park spot.
     */
    private Command parkOtherEnd(boolean fromAudience) {
        double heading = Math.toRadians(PARK_HEADING_DEG);
        double side = fromAudience ? 1.0 : -1.0;
        double parkX = parkX(fromAudience);
        Pose parkPose = new Pose(parkX, parkY(), heading);
        double laneY = LOADING_ZONE_MAX_Y + PARK_LANE_OFF_ZONE_IN;
        final boolean[] backedOff = {false};
        logEvent(String.format(Locale.US, "park on the %s seam at (%.1f, %.1f): back off %.0f in, along the lane at Y %.1f, then in",
                fromAudience ? "A4 / A5" : "A5 / A6", parkX, parkY(), PARK_BACKOFF_IN, laneY));
        return sequential(
                watcher.follow("park: the lane", () -> {
                    // red frame points; the back-off keeps the heading, the next leg turns to the park heading
                    Pose here = red(follower.pose());
                    List<Pose> pts = new ArrayList<>();
                    pts.add(here);
                    if (!backedOff[0]) {   // only the first plan: on a retry he is already off
                        backedOff[0] = true;
                        // straight out from the red wall: robots parked along it are what blocked him
                        pts.add(new Pose(here.x(), here.y() + PARK_BACKOFF_IN, here.heading()));
                    }
                    int turnLeg = pts.size() - 1;   // the first leg after the back-off
                    Pose last = pts.get(pts.size() - 1);
                    if (last.y() < laneY - 1.0) {   // nearer the red wall than the lane: out to it first
                        pts.add(new Pose(last.x(), laneY, heading));
                    }
                    for (Pose q : new Pose[]{new Pose(parkX, laneY, heading), new Pose(parkX, parkY() + PARK_LANE_RISE_STOP_IN, heading)}) {
                        Pose prev = pts.get(pts.size() - 1);
                        if (Math.hypot(q.x() - prev.x(), q.y() - prev.y()) > 1.0) {   // no tiny legs
                            pts.add(q);
                        }
                    }
                    if (pts.size() == 1) {   // already at the rise's end: straight in
                        pts.add(parkPose);
                    }
                    Path[] legs = new Path[pts.size() - 1];
                    for (int i = 0; i < legs.length; i++) {
                        Pose a = field(pts.get(i)), b = field(pts.get(i + 1));
                        legs[i] = i < turnLeg ? line(a, b).constant(a)
                                : i == turnLeg ? line(a, b).linear(a, b) : line(a, b).constant(b);
                    }
                    return legs.length == 1 ? legs[0] : path(legs);
                }, PathWatcher.RETRIES, null, () -> inLoadingZone(red(follower.pose()), PARK_STOP_OVERLAP_IN)),
                lazy(() -> {
                    if (!watcher.gaveUp() || inLoadingZone(red(follower.pose()))) {
                        return parkFinish(parkX, side, parkPose);
                    }
                    return instant(() -> {
                        follower.manual(DrivePowers.zero());
                        logEvent("park: the other end is blocked too, NOT in the LOADING ZONE at " + Drawing.format(follower.pose()));
                    });
                }));
    }

    /** The last park stretch along the wall, then the blocked-park fallbacks. */
    private Command parkFinish(double parkX, double side, Pose parkPose) {
        return sequential(
                watcher.creep("park", () -> field(parkPose), PARK_SPEED_IPS),
                lazy(() -> {
                    if (!watcher.gaveUp()) {
                        return instant(() -> logEvent("parked at " + Drawing.format(follower.pose())));
                    }
                    // Only a shallower spot: never deeper into the zone (that pushes the partner out)
                    List<Double> spots = new ArrayList<>();
                    spots.add(parkX + side * PARK_ALT_OFFSET_IN);
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

    /** John comes to the LOADING ZONE from its audience-side end (red frame: X beyond its middle). */
    private static boolean fromAudienceSide(Pose red) {
        return red.x() >= (LOADING_ZONE_MIN_X + LOADING_ZONE_MAX_X) / 2.0;
    }

    /** The park spot along the wall (red frame): on the seam of the end John comes from. */
    private static double parkX(boolean fromAudience) {
        return fromAudience ? LOADING_ZONE_MAX_X - PARK_INTO_ZONE_IN : LOADING_ZONE_MIN_X + PARK_INTO_ZONE_IN;
    }

    /** The park line (red frame): the robot's half extent across the wall at the park heading, plus PARK_WALL_GAP_IN. */
    private static double parkY() {
        double h = Math.toRadians(PARK_HEADING_DEG);
        double halfAcross = RobotGeometry.LENGTH_IN / 2.0 * Math.abs(Math.sin(h))
                + RobotGeometry.WIDTH_IN / 2.0 * Math.abs(Math.cos(h));
        return RED_WALL_Y + halfAcross + PARK_WALL_GAP_IN;
    }

    /** True when the robot (red frame) overlaps the red LOADING ZONE by at least PARK_OVERLAP_IN both ways. */
    private static boolean inLoadingZone(Pose red) {
        return inLoadingZone(red, PARK_OVERLAP_IN);
    }

    /** True when the robot (red frame) overlaps the red LOADING ZONE by at least {@code overlapIn} both ways. */
    private static boolean inLoadingZone(Pose red, double overlapIn) {
        double c = Math.abs(Math.cos(red.heading()));
        double s = Math.abs(Math.sin(red.heading()));
        double halfX = RobotGeometry.LENGTH_IN / 2.0 * c + RobotGeometry.WIDTH_IN / 2.0 * s;
        double halfY = RobotGeometry.LENGTH_IN / 2.0 * s + RobotGeometry.WIDTH_IN / 2.0 * c;
        double overlapX = Math.min(red.x() + halfX, LOADING_ZONE_MAX_X) - Math.max(red.x() - halfX, LOADING_ZONE_MIN_X);
        double overlapY = Math.min(red.y() + halfY, LOADING_ZONE_MAX_Y) - Math.max(red.y() - halfY, RED_WALL_Y);
        return overlapX >= overlapIn && overlapY >= overlapIn;
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
        Pose red = red(from);
        boolean fromAudience = parkFromAudience(red);
        if (fromAudience != fromAudienceSide(red)) {   // the other end, by the lane (parkOtherEnd)
            double laneY = LOADING_ZONE_MAX_Y + PARK_LANE_OFF_ZONE_IN;
            double length = PARK_BACKOFF_IN + Math.abs(red.y() - laneY) + Math.abs(red.x() - parkX(fromAudience))
                    + Math.abs(laneY - parkY());
            return length / AVG_DRIVE_IPS + PARK_OVERHEAD_S;
        }
        double parkX = parkX(fromAudience);
        double side = fromAudience ? 1.0 : -1.0;
        Pose approach = field(new Pose(parkX + side * PARK_APPROACH_OUT_IN, parkY() + PARK_APPROACH_OFF_WALL_IN,
                Math.toRadians(PARK_HEADING_DEG)));
        Pose park = field(new Pose(parkX, parkY(), Math.toRadians(PARK_HEADING_DEG)));
        return routeLength(from, approach) / AVG_DRIVE_IPS
                + Math.hypot(park.x() - approach.x(), park.y() - approach.y()) / PARK_SPEED_IPS + PARK_OVERHEAD_S;
    }

    /** Estimated time to drive from a to b (field frame) around the HIVE. */
    private double driveSeconds(Pose a, Pose b) {
        return routeLength(a, b) / AVG_DRIVE_IPS + DRIVE_OVERHEAD_S;
    }

    private double routeLength(Pose a, Pose b) {
        List<double[]> route = ShootingZone.squareRoute(a, b, !blue);
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
        double runHeading = Math.toRadians(90.0 + GARDEN_ANGLE_DEG);
        double slowY = gardenSlowY(runHeading, AUDIENCE_WALL_X - extentX(runHeading) - GARDEN_RUN_GAP_IN);
        double end = gardenEnd().y();
        double fast = Math.max(0.0, red(here).y() - slowY);
        double slowLength = Math.max(0.0, Math.min(red(here).y(), slowY) - end);
        double runIps = GARDEN_RUN_SPEED_IPS > 0 ? Math.min(AVG_DRIVE_IPS, GARDEN_RUN_SPEED_IPS) : AVG_DRIVE_IPS;
        return 0.5 + driveSeconds(here, atWall) + fast / runIps + slowLength / COLLECT_SPEED_IPS
                + Intake.SETTLE_MS / 1000.0 + (GARDEN_TOUCH ? 1.0 : 0.0);   // the wall touch and the angling
    }

    /**
     * Where the GARDEN run slows to the collect speed (red frame Y of the center): SLOW_BEFORE_BALL_IN
     * before the robot's footprint, in the strip along the audience wall where the balls lie, reaches
     * the first ball. Angled (120 deg) the farthest-forward corner is the shooter-side one, far from
     * the wall: measuring from it slowed down 7.6" early (r1129-r1135, slow from y -44.8; this gives
     * about -52.4).
     */
    private static double gardenSlowY(double runHeading, double runX) {
        double stripX = AUDIENCE_WALL_X - 2.0 * BALL_RADIUS - 0.5;   // balls lie between here and the wall
        double hl = RobotGeometry.LENGTH_IN / 2.0, hw = RobotGeometry.WIDTH_IN / 2.0;
        double[][] corners = {{-hl, -hw}, {-hl, hw}, {hl, hw}, {hl, -hw}};
        double c = Math.cos(runHeading), s = Math.sin(runHeading);
        double reach = 0.0;   // farthest toward the red wall (-Y) inside the strip, from the center
        for (int e = 0; e < 4; e++) {
            double[] a = corners[e], b = corners[(e + 1) % 4];
            for (int i = 0; i <= 40; i++) {
                double u = a[0] + (b[0] - a[0]) * i / 40.0, v = a[1] + (b[1] - a[1]) * i / 40.0;
                double x = runX + u * c - v * s, dy = u * s + v * c;
                if (x >= stripX) {
                    reach = Math.max(reach, -dy);
                }
            }
        }
        return GARDEN_FIRST_BALL_Y + BALL_RADIUS + reach + SLOW_BEFORE_BALL_IN;
    }

    /** Odometry x minus the real x at the GARDEN wall touch (red frame), 0 without a usable touch. */
    private double gardenFixX = 0.0;
    /** The GARDEN wall touch worked (gardenFixX is from it). */
    private boolean gardenTouched = false;

    /**
     * Strafes square (90, red frame) toward the audience wall at GARDEN_TOUCH_POWER, the heading held,
     * until John stops; there his center is at {@code touchX} (wall - half his width). Sets gardenFixX
     * (ignored beyond GARDEN_TOUCH_MAX_FIX_IN, e.g. a ball in between). The next creep moves him off.
     */
    private Command touchWall(double touchX, double stopY) {
        final ElapsedTime timer = new ElapsedTime();
        final ElapsedTime still = new ElapsedTime();
        final boolean[] done = {false};
        final double[] heading = {0.0};
        final double[] startX = {0.0};
        return new CommandBuilder()
                .setStart(() -> {
                    timer.reset();
                    still.reset();
                    done[0] = false;
                    heading[0] = follower.pose().heading();
                    startX[0] = red(follower.pose()).x();
                })
                .setExecute(() -> {
                    if (done[0]) {
                        return;
                    }
                    Pose p = follower.pose();
                    // Sideways = toward the audience wall (+x in the red frame; blue is rotated 180 deg)
                    double sideways = blue ? -follower.velocity().vx : follower.velocity().vx;
                    if (sideways > GARDEN_TOUCH_STILL_IPS || timer.milliseconds() < GARDEN_TOUCH_GRACE_MS) {
                        still.reset();
                    }
                    boolean stopped = still.milliseconds() >= GARDEN_TOUCH_STILL_MS;
                    boolean atSlow = red(p).y() <= stopY;
                    if (stopped || atSlow || timer.seconds() >= GARDEN_TOUCH_MAX_S) {
                        double odomX = red(p).x();
                        double fix = odomX - touchX;
                        double moved = odomX - startX[0];   // toward the audience wall (+x, red frame)
                        if (stopped && moved < GARDEN_TOUCH_MIN_MOVE_IN) {
                            gardenFixX = 0.0;
                            logEvent(String.format(Locale.US, "garden: wall touch not used (moved only %.1f in toward "
                                    + "the wall: no strafe, or already against it)", moved));
                        } else if (stopped && Math.abs(fix) <= GARDEN_TOUCH_MAX_FIX_IN) {
                            gardenFixX = fix;
                            gardenTouched = true;
                            logEvent(String.format(Locale.US, "garden: wall touch at odometry x %.1f (wall says %.1f): "
                                    + "GARDEN targets shifted %+.1f in", odomX, touchX, fix));
                        } else {
                            gardenFixX = 0.0;
                            logEvent(String.format(Locale.US, "garden: wall touch not used (%s, odometry x %.1f, wall says "
                                    + "%.1f)", stopped ? "too far off" : atSlow ? "reached the balls first" : "did not stop",
                                    odomX, touchX));
                        }
                        follower.hold(p);
                        done[0] = true;
                        return;
                    }
                    // Diagonal: forward toward the red wall (intake A leads: robot -X) and toward the camera
                    // side (robot -Y = the audience wall at 90 red), the heading held
                    double hErr = normalize(heading[0] - p.heading());
                    // Full push until moving sideways, then hold GARDEN_TOUCH_SIDE_IPS (a tap, not a hit)
                    double side = sideways < 1.0 ? GARDEN_TOUCH_POWER : Math.max(GARDEN_TOUCH_MIN_POWER,
                            Math.min(GARDEN_TOUCH_POWER, GARDEN_TOUCH_POWER - 0.04 * (sideways - GARDEN_TOUCH_SIDE_IPS) - 0.1));
                    follower.manual(new DrivePowers(-GARDEN_TOUCH_FORWARD, -side,
                            Math.max(-0.3, Math.min(0.3, 1.5 * hErr))));
                })
                .setDone(() -> done[0]);
    }

    /** Half the robot's footprint along field X / Y at heading {@code h} (a 17" x 17.25" rectangle). */
    private static double extentX(double h) {
        return RobotGeometry.LENGTH_IN / 2.0 * Math.abs(Math.cos(h)) + RobotGeometry.WIDTH_IN / 2.0 * Math.abs(Math.sin(h));
    }

    private static double extentY(double h) {
        return RobotGeometry.LENGTH_IN / 2.0 * Math.abs(Math.sin(h)) + RobotGeometry.WIDTH_IN / 2.0 * Math.abs(Math.cos(h));
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
     * (ShootingZone.squareRoute): square to the HIVE while passing it, turning to the end heading on
     * the first leg after it (on the first leg when the HIVE is not in the way).
     */
    private Path around(Pose end, double speed) {
        Path p = squarePast(end, false);
        return speed > 0 ? slow(p, speed) : p;
    }

    /**
     * The squareRoute() legs to {@code end}: before the HIVE John turns square to it
     * (ShootingZone.squareHeading), keeps that heading while passing it, then turns toward the CELL
     * ({@code aimed}, Aimer.facingTarget) or to the end heading. With no square legs: aimed the whole
     * way, or turning to the end heading on the first leg.
     */
    private Path squarePast(Pose end, boolean aimed) {
        Pose here = follower.pose();
        List<double[]> route = ShootingZone.squareRoute(here, end, !blue);
        int firstSquare = -1, lastSquare = -1;
        for (int i = 0; i < route.size() - 1; i++) {
            if (route.get(i + 1)[2] > 0) {
                firstSquare = firstSquare < 0 ? i : firstSquare;
                lastSquare = i;
            }
        }
        double sq = ShootingZone.squareHeading(here.heading(), end.heading());
        Path[] legs = new Path[route.size() - 1];
        for (int i = 0; i < legs.length; i++) {
            Pose a = i == 0 ? here : new Pose(route.get(i)[0], route.get(i)[1], end.heading());
            Pose b = new Pose(route.get(i + 1)[0], route.get(i + 1)[1], end.heading());
            if (firstSquare < 0) {
                legs[i] = aimed ? line(a, b).heading(aimer.facingTarget(i == 0 ? here.heading() : null))
                        : i == 0 ? line(a, b).linear(a, b) : line(a, b).constant(b);
            } else if (i < firstSquare) {
                legs[i] = line(a, b).linear(a, b.withHeading(sq));
            } else if (i <= lastSquare) {
                legs[i] = line(a, b).constant(b.withHeading(sq));
            } else if (aimed) {
                legs[i] = line(a, b).heading(aimer.facingTarget(i == lastSquare + 1 ? sq : null));
            } else {
                legs[i] = i == lastSquare + 1 ? line(a, b).linear(a.withHeading(sq), b) : line(a, b).constant(b);
            }
        }
        return legs.length == 1 ? legs[0] : path(legs);
    }

    /**
     * From where the robot is now to {@code end} (field frame) around the HIVE frame, the shooter
     * turning toward the target CELL along the way (Aimer.facingTarget), so it arrives aimed; square
     * to the HIVE while passing it (squarePast), aiming only after it.
     */
    private Path aimedRoute(Pose end) {
        return squarePast(end, true);
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
        Object[] cam = TagLocalizer.logCheck(frame, p);   // odometry check (camera position, odometry heading)
        Flywheel fw = shooter.flywheel();
        log.row(clock.seconds(), step, p.x(), p.y(), Math.toDegrees(p.heading()), follower.mode(),
                shooter.status(), intake.status(), t[0], t[1], tracker.source(), tracker.upCellName(),
                frame == null ? null : frame.ids(), watchdog.state(), shooter.indexer().degrees(),
                shooter.indexer().targetDegrees(), shooter.indexer().amps(), cam[0], cam[1], cam[2],
                tracker.hiveStateName(), Double.isNaN(tracker.upHeight()) ? null : tracker.upHeight(),
                fw.rpm(), fw.isOn() ? fw.targetRpm() : 0.0, fw.mode(), fw.isOn() ? fw.amps() : null,
                fw.effectiveF(), fw.voltage(), event);
    }

    private void logEvent(String event) {
        logRow(event);
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
