package org.firstinspires.ftc.teamcode.John.Dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.John.RobotGeometry;

import java.util.Locale;

/**
 * FTC Dashboard field overlay for John, in FTC field coordinates: origin at the field center, +X
 * toward the audience wall, +Y toward the blue side, inches, heading counter-clockwise from +X.
 *
 * Per loop: drawField() once, then drawRobot() / drawRobotOutline() for the poses to show.
 * Live-editable from the Dashboard config panel as "JohnDrawing".
 */
@Config("JohnDrawing")
public class Drawing {

    /**
     * Turns the view 180 degrees (the blue side shows where the red side was). Display only;
     * coordinates are unchanged.
     */
    public static boolean BLUE_PERSPECTIVE = false;

    public static boolean USE_CUSTOM_IMAGE = true;
    public static String FIELD_IMAGE_PATH = "/images/BIOBUZZ_field_2026.webp";
    public static double FIELD_SIZE_IN = 144.0;

    public static boolean DRAW_AXES = true;
    public static double AXIS_LENGTH = 24.0;
    public static String X_AXIS_COLOR = "red";
    public static String Y_AXIS_COLOR = "green";

    public static boolean DRAW_GRID_POINTS = true;
    /** Tile corners. */
    public static double GRID_STEP = 24.0;
    public static double GRID_DOT_RADIUS = 0.4;
    public static String GRID_DOT_COLOR = "white";

    /** John's frame, inches (length along the heading). */
    public static double ROBOT_LENGTH = RobotGeometry.LENGTH_IN;
    public static double ROBOT_WIDTH = RobotGeometry.WIDTH_IN;
    /**
     * The robot is drawn as a picture of a mecanum frame (the same as Barry's), which has a heading
     * arrow in it. Without the picture it is an outline with a heading line.
     */
    public static boolean USE_ROBOT_IMAGE = true;
    public static String ROBOT_IMAGE_PATH = "/images/robot.png";
    /** Turns the picture if its arrow does not point along +X of the picture. */
    public static double ROBOT_IMAGE_OFFSET_ROTATION_DEG = 0.0;
    /** A short line out of the shooter side, so the robot's orientation is clear on the field. */
    public static boolean DRAW_SHOOTER = true;
    public static String SHOOTER_COLOR = "cyan";
    public static String ROBOT_COLOR = "white";
    public static String HEADING_COLOR = "pink";
    public static int OUTLINE_STROKE_WIDTH = 1;
    public static int HEADING_STROKE_WIDTH = 2;
    public static String TEXT_FONT = "3px sans-serif";

    /** The Dashboard's own field is drawn when the custom image is off (pass to new TelemetryPacket(...)). */
    public static boolean drawDefaultField() {
        return !USE_CUSTOM_IMAGE;
    }

    /** Field image, axes and tile-corner dots. */
    public static void drawField(Canvas canvas) {
        if (canvas == null) {
            return;
        }
        if (USE_CUSTOM_IMAGE) {
            drawFieldImage(canvas);
        }
        applyFieldTransform(canvas);
        if (DRAW_GRID_POINTS) {
            double half = FIELD_SIZE_IN / 2.0;
            canvas.setFill(GRID_DOT_COLOR);
            for (double x = -half; x <= half + 1e-6; x += GRID_STEP) {
                for (double y = -half; y <= half + 1e-6; y += GRID_STEP) {
                    canvas.fillCircle(x, y, GRID_DOT_RADIUS);
                }
            }
        }
        if (DRAW_AXES) {
            canvas.setStrokeWidth(1);
            canvas.setStroke(X_AXIS_COLOR);
            canvas.strokeLine(0, 0, AXIS_LENGTH, 0);
            canvas.setStroke(Y_AXIS_COLOR);
            canvas.strokeLine(0, 0, 0, AXIS_LENGTH);
        }
    }

    /** The robot at a pose: the frame picture (or footprint and heading line), and the shooter side. */
    public static void drawRobot(Canvas canvas, Pose pose) {
        if (canvas == null || pose == null) {
            return;
        }
        if (USE_ROBOT_IMAGE && ROBOT_IMAGE_PATH != null && !ROBOT_IMAGE_PATH.isEmpty()) {
            applyFieldTransform(canvas);
            // Images are drawn with a flipped Y, so the angle is negated (same as Barry's Drawing)
            double rotation = pose.heading() + Math.toRadians(ROBOT_IMAGE_OFFSET_ROTATION_DEG);
            canvas.drawImage(ROBOT_IMAGE_PATH, pose.x(), pose.y(), ROBOT_LENGTH, ROBOT_WIDTH, -rotation,
                    ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0, false);
        } else {
            drawFootprint(canvas, pose, ROBOT_COLOR, HEADING_COLOR, null);
        }
        if (DRAW_SHOOTER) {
            double dir = RobotGeometry.shooterDirection(pose.heading());
            double from = ROBOT_WIDTH / 2.0;
            canvas.setStroke(SHOOTER_COLOR);
            canvas.setStrokeWidth(HEADING_STROKE_WIDTH);
            canvas.strokeLine(pose.x() + from * Math.cos(dir), pose.y() + from * Math.sin(dir),
                    pose.x() + (from + 4.0) * Math.cos(dir), pose.y() + (from + 4.0) * Math.sin(dir));
        }
    }

    /** Another pose (e.g. a target) as a footprint in one color, with an optional label. */
    public static void drawRobotOutline(Canvas canvas, Pose pose, String color, String label) {
        drawFootprint(canvas, pose, color, color, label);
    }

    private static void drawFootprint(Canvas canvas, Pose pose, String bodyColor, String headingColor, String label) {
        if (canvas == null || pose == null) {
            return;
        }
        applyFieldTransform(canvas);
        double x = pose.x();
        double y = pose.y();
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());
        double halfLength = ROBOT_LENGTH / 2.0;
        double halfWidth = ROBOT_WIDTH / 2.0;

        double[] cornersX = {halfLength, halfLength, -halfLength, -halfLength};
        double[] cornersY = {halfWidth, -halfWidth, -halfWidth, halfWidth};
        double[] xs = new double[4];
        double[] ys = new double[4];
        for (int i = 0; i < 4; i++) {
            xs[i] = x + cornersX[i] * cos - cornersY[i] * sin;
            ys[i] = y + cornersX[i] * sin + cornersY[i] * cos;
        }
        canvas.setStroke(bodyColor);
        canvas.setStrokeWidth(OUTLINE_STROKE_WIDTH);
        canvas.strokePolygon(xs, ys);
        canvas.setStroke(headingColor);
        canvas.setStrokeWidth(HEADING_STROKE_WIDTH);
        canvas.strokeLine(x, y, x + (halfLength + 4.0) * cos, y + (halfLength + 4.0) * sin);

        if (label != null && !label.isEmpty()) {
            double[] at = screenOffset(x, y, halfWidth + 1.0, halfWidth + 1.0);
            canvas.setFill(bodyColor);
            canvas.fillText(label, at[0], at[1], TEXT_FONT, textTheta(), false);
        }
    }

    // ---- Canvas mapping ----

    /** Canvas rotation for FTC coordinates (plus BLUE_PERSPECTIVE), radians. */
    private static double rotation() {
        return Math.PI + (BLUE_PERSPECTIVE ? Math.PI : 0.0);
    }

    private static void applyFieldTransform(Canvas canvas) {
        canvas.setRotation(rotation()).setTranslation(0, 0);
    }

    /**
     * The field image is drawn from its corner, so it gets its own placement: a quarter turn from
     * the FTC axes and shifted by half a field, which puts its center on the FTC origin.
     */
    private static void drawFieldImage(Canvas canvas) {
        double half = FIELD_SIZE_IN / 2.0;
        double tx = BLUE_PERSPECTIVE ? half : -half;
        double ty = BLUE_PERSPECTIVE ? -half : half;
        canvas.setRotation(Math.toRadians(-90.0) + (BLUE_PERSPECTIVE ? Math.PI : 0.0)).setTranslation(tx, ty);
        canvas.drawImage(FIELD_IMAGE_PATH, half, half, FIELD_SIZE_IN, FIELD_SIZE_IN, 0.0, half, half, false);
    }

    /** A point moved by (right, up) inches on screen, in FTC coordinates (keeps labels beside things). */
    private static double[] screenOffset(double x, double y, double right, double up) {
        // On the canvas (before our transform) screen up = +X and screen right = -Y
        double cx = up;
        double cy = -right;
        double a = -rotation();
        return new double[]{
                x + cx * Math.cos(a) - cy * Math.sin(a),
                y + cx * Math.sin(a) + cy * Math.cos(a)};
    }

    /** Text angle that keeps labels upright on screen. */
    private static double textTheta() {
        return rotation() + Math.toRadians(90.0);
    }

    /** "x, y, heading" in FTC coordinates, for telemetry. */
    public static String format(Pose pose) {
        return String.format(Locale.US, "(%.1f, %.1f) in, %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
