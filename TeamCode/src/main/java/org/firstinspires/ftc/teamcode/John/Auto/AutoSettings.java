package org.firstinspires.ftc.teamcode.John.Auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.John.RobotGeometry;

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
 * The final pose of the autonomous can be handed to a following TeleOp (savePose / loadPose).
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
    }

    /** Hands the pose (FTC) to a following OpMode, e.g. a TeleOp. */
    public static void savePose(Map<String, Object> blackboard, Pose pose) {
        blackboard.put(KEY_POSE_X, pose.x());
        blackboard.put(KEY_POSE_Y, pose.y());
        blackboard.put(KEY_POSE_H, pose.heading());
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
