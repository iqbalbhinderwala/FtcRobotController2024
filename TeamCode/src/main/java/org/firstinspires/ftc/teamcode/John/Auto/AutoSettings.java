package org.firstinspires.ftc.teamcode.John.Auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.John.RobotGeometry;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.IOException;
import java.util.Locale;
import java.util.Map;

/**
 * Autonomous settings: alliance, start tile and start delay. They are chosen in [John] Game Setup
 * (a separate OpMode, run before the match) and kept in the SDK blackboard; [John] Auto only reads
 * them, so nothing can be changed by a stray button press while it waits in INIT, and it refuses to
 * run without them (isSaved). The blackboard lasts until the Robot Controller app restarts.
 *
 * Game Setup: call update(gamepad1) and save(blackboard) every loop, addTelemetry(telemetry, true).
 * gamepad1:
 *   B / X                red / blue alliance
 *   dpad left / right    previous / next start tile
 *   left / right bumper  start delay - / + 1 s
 *
 * The autonomous hands its last pose and which CELL is up to the TeleOp (savePose / loadPose /
 * poseAgeS, saveUpAudience / loadUpAudience); the time stamp lets the TeleOp ignore a pose from an
 * earlier run.
 *
 * The blackboard is cleared when the Robot Controller app restarts (also by every redeploy), so
 * save() also writes the choice to SETUP_FILE; the TeleOp falls back to it (loadFile), so it keeps
 * the right alliance after a restart (2026-10-24, r0171 ran as red that way). [John] Auto does not:
 * it still needs a Game Setup in the current app session.
 *
 * Poses are FTC field coordinates (origin at the field center, +X toward the audience, +Y toward
 * blue, inches, heading counter-clockwise from +X). Red starts with John's camera wall flush on the
 * audience wall and the shooter (his left) facing away from it toward the HIVE, so the heading is
 * +90 degrees; blue is the same reflected through the field center.
 */
public class AutoSettings {

    public enum Alliance { RED, BLUE }

    /** Start tiles along the audience wall on the red side (Competition Manual 9.4). */
    public enum StartTile {
        B1(-36.0), C1(-12.0);

        /** FTC Y of the tile center on the red side. */
        public final double redY;

        StartTile(double redY) {
            this.redY = redY;
        }
    }

    public static final double MAX_DELAY_S = 10.0;

    private static final String KEY_ALLIANCE = "john.auto.alliance";
    private static final String KEY_TILE = "john.auto.tile";
    private static final String KEY_DELAY = "john.auto.delay";
    private static final String KEY_POSE_X = "john.pose.x";
    private static final String KEY_POSE_Y = "john.pose.y";
    private static final String KEY_POSE_H = "john.pose.heading";
    private static final String KEY_POSE_TIME = "john.pose.time";
    private static final String KEY_UP_AUDIENCE = "john.upAudience";

    private Alliance alliance = Alliance.RED;
    private StartTile tile = StartTile.C1;
    private double delayS = 0.0;

    private boolean firstUpdate = true;

    /** Starts from the last choice kept in the blackboard (red, C1, no delay if there is none). */
    public AutoSettings(Map<String, Object> blackboard) {
        try {
            Object a = blackboard.get(KEY_ALLIANCE);
            if (a instanceof String) alliance = Alliance.valueOf((String) a);
            Object t = blackboard.get(KEY_TILE);
            if (t instanceof String) tile = StartTile.valueOf((String) t);
            Object d = blackboard.get(KEY_DELAY);
            if (d instanceof Number) delayS = clampDelay(((Number) d).doubleValue());
        } catch (IllegalArgumentException ignored) {
            // an unknown stored value: keep the defaults
        }
    }

    /** True when a Game Setup has been saved (since the Robot Controller app started). */
    public static boolean isSaved(Map<String, Object> blackboard) {
        try {
            Object a = blackboard.get(KEY_ALLIANCE);
            Object t = blackboard.get(KEY_TILE);
            Object d = blackboard.get(KEY_DELAY);
            if (!(a instanceof String) || !(t instanceof String) || !(d instanceof Number)) {
                return false;
            }
            Alliance.valueOf((String) a);
            StartTile.valueOf((String) t);
            return true;
        } catch (IllegalArgumentException e) {
            return false;
        }
    }

    /** Reads the button presses (the SDK's xWasPressed(): one action per press). Call once per loop. */
    public void update(Gamepad pad) {
        if (firstUpdate) {
            pad.resetEdgeDetection();   // ignore presses from before this OpMode
            firstUpdate = false;
        }
        if (pad.bWasPressed()) alliance = Alliance.RED;
        if (pad.xWasPressed()) alliance = Alliance.BLUE;
        StartTile[] tiles = StartTile.values();
        if (pad.dpadLeftWasPressed()) tile = tiles[(tile.ordinal() + tiles.length - 1) % tiles.length];
        if (pad.dpadRightWasPressed()) tile = tiles[(tile.ordinal() + 1) % tiles.length];
        if (pad.leftBumperWasPressed()) delayS = clampDelay(delayS - 1.0);
        if (pad.rightBumperWasPressed()) delayS = clampDelay(delayS + 1.0);
    }

    /** Shows the settings; withControls adds the button hints (Game Setup). */
    public void addTelemetry(Telemetry telemetry, boolean withControls) {
        Pose p = startPose();
        telemetry.addData("Alliance", withControls ? "%s   (B red, X blue)" : "%s", alliance);
        telemetry.addData("Start tile", withControls ? "%s   (dpad left / right)" : "%s", tile);
        telemetry.addData("Start delay", withControls ? "%.0f s   (bumpers - / +)" : "%.0f s", delayS);
        telemetry.addData("Start pose (FTC)", "(%.1f, %.1f) in, %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addLine("Place John with the camera wall flush on the audience wall, shooter facing the field.");
    }

    /** Start pose for the current choice (FTC coordinates). */
    public Pose startPose() {
        return startPose(alliance, tile);
    }

    public static Pose startPose(Alliance alliance, StartTile tile) {
        // Camera wall flush on the audience wall (X = 72): the robot's width is along field X
        double x = 72.0 - RobotGeometry.WIDTH_IN / 2.0;
        double heading = Math.toRadians(90.0);   // shooter (left side) faces away from the wall
        return alliance == Alliance.RED
                ? new Pose(x, tile.redY, heading)
                : new Pose(-x, -tile.redY, heading + Math.PI);   // reflected through the field center
    }

    public Alliance alliance() {
        return alliance;
    }

    public StartTile tile() {
        return tile;
    }

    public double delayS() {
        return delayS;
    }

    /** Keeps the choice for the next INIT. */
    public void save(Map<String, Object> blackboard) {
        blackboard.put(KEY_ALLIANCE, alliance.name());
        blackboard.put(KEY_TILE, tile.name());
        blackboard.put(KEY_DELAY, delayS);
        String line = alliance.name() + "," + tile.name() + "," + delayS;
        if (!line.equals(lastWritten)) {   // Game Setup saves every loop: write only on a change
            try (FileWriter w = new FileWriter(setupFile())) {
                w.write(line + "," + System.currentTimeMillis() + "\n");
                lastWritten = line;
            } catch (IOException ignored) {
                // the blackboard still has it
            }
        }
    }

    private String lastWritten;

    private static File setupFile() {
        return new File(AppUtil.FIRST_FOLDER, "john_game_setup.txt");
    }

    /**
     * The last Game Setup from the file (survives app restarts), or null. Its age in seconds is in
     * fileAgeS().
     */
    public static AutoSettings loadFile() {
        try (BufferedReader r = new BufferedReader(new FileReader(setupFile()))) {
            String[] parts = r.readLine().trim().split(",");
            AutoSettings s = new AutoSettings(new java.util.HashMap<>());
            s.alliance = Alliance.valueOf(parts[0]);
            s.tile = StartTile.valueOf(parts[1]);
            s.delayS = clampDelay(Double.parseDouble(parts[2]));
            s.fileTimeMs = parts.length > 3 ? Long.parseLong(parts[3]) : 0L;
            return s;
        } catch (IOException | RuntimeException e) {
            return null;
        }
    }

    private long fileTimeMs;

    /** For a loadFile() result: how long ago it was saved, for telemetry. */
    public String fileAge() {
        double s = (System.currentTimeMillis() - fileTimeMs) / 1000.0;
        return fileTimeMs <= 0 ? "unknown age" : s < 120 ? String.format(Locale.US, "%.0f s ago", s)
                : s < 7200 ? String.format(Locale.US, "%.0f min ago", s / 60) : String.format(Locale.US, "%.1f h ago", s / 3600);
    }

    /** Hands the pose (FTC) to a following OpMode, e.g. a TeleOp, with the time it was saved. */
    public static void savePose(Map<String, Object> blackboard, Pose pose) {
        blackboard.put(KEY_POSE_X, pose.x());
        blackboard.put(KEY_POSE_Y, pose.y());
        blackboard.put(KEY_POSE_H, pose.heading());
        blackboard.put(KEY_POSE_TIME, System.currentTimeMillis());
    }

    /** Seconds since the pose was saved, or infinity if there is none. */
    public static double poseAgeS(Map<String, Object> blackboard) {
        Object t = blackboard.get(KEY_POSE_TIME);
        return t instanceof Number ? (System.currentTimeMillis() - ((Number) t).longValue()) / 1000.0
                : Double.POSITIVE_INFINITY;
    }

    /** Which CELL of our HIVE was up at the end of the autonomous. */
    public static void saveUpAudience(Map<String, Object> blackboard, boolean upAudience) {
        blackboard.put(KEY_UP_AUDIENCE, upAudience);
    }

    /** The up CELL the autonomous saw last (true = audience), or null. */
    public static Boolean loadUpAudience(Map<String, Object> blackboard) {
        Object v = blackboard.get(KEY_UP_AUDIENCE);
        return v instanceof Boolean ? (Boolean) v : null;
    }

    /** The pose an earlier OpMode handed over, or null. */
    public static Pose loadPose(Map<String, Object> blackboard) {
        Object x = blackboard.get(KEY_POSE_X);
        Object y = blackboard.get(KEY_POSE_Y);
        Object h = blackboard.get(KEY_POSE_H);
        if (x instanceof Number && y instanceof Number && h instanceof Number) {
            return new Pose(((Number) x).doubleValue(), ((Number) y).doubleValue(), ((Number) h).doubleValue());
        }
        return null;
    }

    private static double clampDelay(double s) {
        return Math.max(0.0, Math.min(MAX_DELAY_S, s));
    }
}
