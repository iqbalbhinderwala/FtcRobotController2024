package org.firstinspires.ftc.teamcode.Barry.Dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import java.util.Locale;

/**
 * Utility class for rendering robot positions, background images, coordinate axes, and grid lattice points on the FTC Dashboard Field Overlay.
 * Configurable via FTC Dashboard live config.
 */
@Config
public class Drawing {

    public enum CoordinateConvention {
        FTC,    // Origin at the field center, +X toward the audience wall, +Y toward the blue side
        PEDRO   // Origin at the red / audience wall corner, +X toward the blue side, +Y away from the audience wall (0 to 144)
    }

    // Configurable Dashboard Settings
    public static CoordinateConvention CONVENTION = CoordinateConvention.PEDRO;
    /**
     * Reflects the whole canvas (image, axes, robot, overlays) through the field center, i.e. turns
     * the view 180 degrees: the blue side shows where the red side was. Display only; coordinates
     * are unchanged.
     */
    public static boolean BLUE_PERSPECTIVE = false;

    public static boolean DRAW_FIELD = true;
    public static boolean DRAW_AXES = true;
    public static boolean DRAW_GRID_POINTS = true;
    public static boolean USE_CUSTOM_IMAGE = true;

    public static String FIELD_IMAGE_PATH = "/images/BIOBUZZ_field_2026.webp";
    public static double FIELD_IMAGE_WIDTH = 144.0;
    public static double FIELD_IMAGE_HEIGHT = 144.0;
    /** Rotates only the field background image about its center, degrees counter-clockwise on screen. */
    public static double FIELD_IMAGE_ROTATION_DEG = 0.0;

    public static double GRID_STEP = 24.0;             // Grid step in inches
    public static double GRID_DOT_RADIUS = 0.2;        // Radius of lattice dots in inches
    public static String GRID_DOT_COLOR = "red";       // Dot fill color
    public static String GRID_DOT_STROKE = "white";    // Dot border color
    public static boolean SHOW_GRID_LABELS = false;    // Show (x,y) text next to dots
    public static String GRID_TEXT_COLOR = "cyan";     // Coordinate text color
    public static String GRID_TEXT_FONT = "2px sans-serif"; // Coordinate text font
    public static double GRID_TEXT_ROTATION_DEG = 0.0; // Text rotation in degrees

    public static boolean USE_ROBOT_IMAGE = true;
    public static String ROBOT_IMAGE_PATH = "/images/robot.png";
    public static double ROBOT_IMAGE_OFFSET_ROTATION_DEG = 0.0; // Adjustment angle if robot image is rotated
    public static boolean DRAW_ROBOT_OUTLINE = true;            // Draw outline and heading line on top of image
    public static double ROBOT_ALPHA = 1.0;

    public static double ROBOT_WIDTH = 18.0;
    public static double ROBOT_LENGTH = 17.0;
    public static double AXIS_LENGTH = 40.0;

    public static int ROBOT_OUTLINE_STROKE_WIDTH = 1;  // Stroke width for robot body outline
    public static int ROBOT_HEADING_STROKE_WIDTH = 2;  // Stroke width for heading vector line
    public static int AXIS_STROKE_WIDTH = 1;           // Stroke width for coordinate axes

    public static String ROBOT_BODY_COLOR = "white";
    public static String ROBOT_HEADING_COLOR = "orange";
    public static String X_AXIS_COLOR = "red";
    public static String Y_AXIS_COLOR = "green";

    /**
     * Primary entry point for rendering to the FTC Dashboard Field Overlay.
     * Automatically handles field background, grid points, coordinate axes, and robot drawing based on live @Config settings.
     *
     * @param fieldOverlay The FTC Dashboard Canvas
     * @param pose         The robot's current pose
     */
    public static void drawRobot(Canvas fieldOverlay, Pose pose) {
        if (fieldOverlay == null) {
            return;
        }

        // Draw custom background field if enabled
        if (DRAW_FIELD && USE_CUSTOM_IMAGE) {
            drawFieldInternal(fieldOverlay, CONVENTION);
        }

        // Draw robot body, image, and heading line
        if (pose != null) {
            fieldOverlay.setAlpha(ROBOT_ALPHA);
            drawRobotInternal(fieldOverlay, pose, ROBOT_WIDTH, ROBOT_LENGTH, ROBOT_BODY_COLOR, ROBOT_HEADING_COLOR, CONVENTION);
            fieldOverlay.setAlpha(1.0);
        }

        // Draw coordinate axes if enabled
        if (DRAW_AXES) {
            drawAxesInternal(fieldOverlay, AXIS_LENGTH, CONVENTION);
        }

        // Draw grid lattice points if enabled
        if (DRAW_GRID_POINTS) {
            drawGridPointsInternal(fieldOverlay, CONVENTION);
        }
    }

    /**
     * Alias for drawRobot.
     *
     * @param fieldOverlay The FTC Dashboard Canvas
     * @param pose         The robot's current pose
     */
    public static void draw(Canvas fieldOverlay, Pose pose) {
        drawRobot(fieldOverlay, pose);
    }

    /**
     * Draws an AprilTag cluster: a line along the sticker with a dot at each of its 4 tags.
     * Tag offsets from the cluster center are from Competition Manual Figure 9-15.
     *
     * @param x      cluster center X in inches (Pedro frame)
     * @param y      cluster center Y in inches (Pedro frame)
     * @param dirX   unit vector along the sticker, X component (Pedro frame)
     * @param dirY   unit vector along the sticker, Y component (Pedro frame)
     * @param color  Dashboard color string
     * @param label  text drawn next to the center
     */
    public static void drawCluster(Canvas fieldOverlay, double x, double y, double dirX, double dirY,
                                   String color, String label) {
        if (fieldOverlay == null) {
            return;
        }
        applyFieldTransform(fieldOverlay, CONVENTION);

        final double[] tagOffsets = {-6.50, -2.75, 2.75, 6.50};
        final double halfLength = 6.50 + 3.25 / 2.0;

        fieldOverlay.setStroke(color);
        fieldOverlay.setStrokeWidth(2);
        fieldOverlay.strokeLine(x - dirX * halfLength, y - dirY * halfLength,
                x + dirX * halfLength, y + dirY * halfLength);

        fieldOverlay.setFill(color);
        for (double offset : tagOffsets) {
            fieldOverlay.fillCircle(x + dirX * offset, y + dirY * offset, 1.0);
        }

        fieldOverlay.setStroke("white");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeCircle(x, y, 1.5);

        if (label != null && !label.isEmpty()) {
            fieldOverlay.setFill(color);
            double[] at = screenOffset(CONVENTION, x, y, 2.0, 2.0);
            fieldOverlay.fillText(label, at[0], at[1], GRID_TEXT_FONT, textTheta(CONVENTION, 0.0), false);
        }
    }

    /**
     * Draws a single AprilTag as a filled rectangle with a label (top-down view).
     *
     * @param x       tag center X in inches (Pedro frame)
     * @param y       tag center Y in inches (Pedro frame)
     * @param dirX    unit vector along the tag width, X component (Pedro frame)
     * @param dirY    unit vector along the tag width, Y component (Pedro frame)
     * @param width   extent along the direction, inches (3.25 for a tag seen face-on)
     * @param height  extent perpendicular to the direction, inches (foreshortened when the tag is tilted)
     * @param color   Dashboard color string
     * @param label   text drawn next to the tag (null for none)
     */
    public static void drawTag(Canvas fieldOverlay, double x, double y, double dirX, double dirY,
                               double width, double height, String color, String label) {
        if (fieldOverlay == null) {
            return;
        }
        applyFieldTransform(fieldOverlay, CONVENTION);

        double hw = width / 2.0;
        double hh = height / 2.0;
        double perpX = -dirY;
        double perpY = dirX;

        double[] xs = {
                x + dirX * hw + perpX * hh,
                x + dirX * hw - perpX * hh,
                x - dirX * hw - perpX * hh,
                x - dirX * hw + perpX * hh};
        double[] ys = {
                y + dirY * hw + perpY * hh,
                y + dirY * hw - perpY * hh,
                y - dirY * hw - perpY * hh,
                y - dirY * hw + perpY * hh};

        fieldOverlay.setFill(color);
        fieldOverlay.fillPolygon(xs, ys);
        fieldOverlay.setStroke("white");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokePolygon(xs, ys);

        if (label != null && !label.isEmpty()) {
            fieldOverlay.setFill("white");
            double[] at = screenOffset(CONVENTION, x, y, -1.0, hh + 1.5);
            fieldOverlay.fillText(label, at[0], at[1], GRID_TEXT_FONT, textTheta(CONVENTION, 0.0), false);
        }
    }

    // =========================================================================
    // Internal Drawing Helpers
    // =========================================================================

    /** Canvas rotation of a convention, without BLUE_PERSPECTIVE (radians). */
    private static double conventionRotation(CoordinateConvention convention) {
        return convention == CoordinateConvention.PEDRO ? Math.toRadians(-90.0) : Math.toRadians(180.0);
    }

    /** Canvas rotation of a convention, including BLUE_PERSPECTIVE (radians). */
    private static double totalRotation(CoordinateConvention convention) {
        return conventionRotation(convention) + (BLUE_PERSPECTIVE ? Math.PI : 0.0);
    }

    /**
     * Maps the convention's coordinates onto the Dashboard canvas. Both conventions put the field
     * image in the same place on screen; only the axes differ. FTC = Pedro rotated 90 degrees:
     * ftcX = 72 - pedroY, ftcY = pedroX - 72, which on the canvas is a 180 degree rotation.
     * BLUE_PERSPECTIVE then reflects everything through the field center (canvas origin).
     */
    private static void applyFieldTransform(Canvas fieldOverlay, CoordinateConvention convention) {
        double tx = (convention == CoordinateConvention.PEDRO) ? -72.0 : 0.0;
        double ty = (convention == CoordinateConvention.PEDRO) ? 72.0 : 0.0;
        if (BLUE_PERSPECTIVE) {
            tx = -tx;
            ty = -ty;
        }
        fieldOverlay
                .setRotation(totalRotation(convention))
                .setTranslation(tx, ty);
    }

    /**
     * A point moved by (right, up) inches on screen, in the convention's coordinates. Keeps labels
     * at the same place next to what they label whatever the convention and perspective.
     */
    private static double[] screenOffset(CoordinateConvention convention, double x, double y,
                                         double right, double up) {
        // On the canvas (before our transform) screen up = +X and screen right = -Y
        double cx = up;
        double cy = -right;
        // Undo the convention's total rotation to express that vector in its coordinates
        double a = -totalRotation(convention);
        return new double[]{
                x + cx * Math.cos(a) - cy * Math.sin(a),
                y + cx * Math.sin(a) + cy * Math.cos(a)};
    }

    /** Text angle that keeps labels upright on screen whatever the convention and perspective. */
    private static double textTheta(CoordinateConvention convention, double theta) {
        // Labels are upright in Pedro from the red perspective; compensate for any extra canvas rotation
        return theta + totalRotation(convention) - conventionRotation(CoordinateConvention.PEDRO);
    }

    private static void drawFieldInternal(Canvas fieldOverlay, CoordinateConvention convention) {
        // The image is laid out in the Pedro frame; draw it the same way for every convention so it
        // never moves on screen when switching conventions.
        // Anchored at the image center so FIELD_IMAGE_ROTATION_DEG turns it in place. The image is drawn
        // with a flipped Y, so a positive canvas angle turns it clockwise on screen; negate for CCW.
        applyFieldTransform(fieldOverlay, CoordinateConvention.PEDRO);
        fieldOverlay.drawImage(
                FIELD_IMAGE_PATH,
                FIELD_IMAGE_WIDTH / 2.0, 144.0 - FIELD_IMAGE_HEIGHT / 2.0,
                FIELD_IMAGE_WIDTH, FIELD_IMAGE_HEIGHT,
                -Math.toRadians(FIELD_IMAGE_ROTATION_DEG),
                FIELD_IMAGE_WIDTH / 2.0, FIELD_IMAGE_HEIGHT / 2.0, false
        );
        applyFieldTransform(fieldOverlay, convention);
    }

    private static void drawGridPointsInternal(Canvas fieldOverlay, CoordinateConvention convention) {
        applyFieldTransform(fieldOverlay, convention);

        double minX = (convention == CoordinateConvention.PEDRO) ? 0.0 : -72.0;
        double maxX = (convention == CoordinateConvention.PEDRO) ? 144.0 : 72.0;
        double minY = (convention == CoordinateConvention.PEDRO) ? 0.0 : -72.0;
        double maxY = (convention == CoordinateConvention.PEDRO) ? 144.0 : 72.0;

        fieldOverlay
                .setStroke(GRID_DOT_STROKE)
                .setStrokeWidth(1);

        for (double x = minX; x <= maxX; x += GRID_STEP) {
            for (double y = minY; y <= maxY; y += GRID_STEP) {
                fieldOverlay.setFill(GRID_DOT_COLOR);
                fieldOverlay.fillCircle(x, y, GRID_DOT_RADIUS);
                fieldOverlay.strokeCircle(x, y, GRID_DOT_RADIUS);

                if (SHOW_GRID_LABELS) {
                    String label = String.format(Locale.US, "(%d,%d)", Math.round(x), Math.round(y));
                    fieldOverlay
                            .setFill(GRID_TEXT_COLOR)
                            .fillText(label, screenOffset(convention, x, y, 1.5, 1.5)[0],
                                    screenOffset(convention, x, y, 1.5, 1.5)[1], GRID_TEXT_FONT,
                                    textTheta(convention, Math.toRadians(GRID_TEXT_ROTATION_DEG)), false);
                }
            }
        }
    }

    private static void drawAxesInternal(Canvas fieldOverlay, double length, CoordinateConvention convention) {
        applyFieldTransform(fieldOverlay, convention);

        fieldOverlay.setStroke(X_AXIS_COLOR);
        fieldOverlay.setStrokeWidth(AXIS_STROKE_WIDTH);
        fieldOverlay.strokeLine(0, 0, length, 0);

        fieldOverlay.setStroke(Y_AXIS_COLOR);
        fieldOverlay.setStrokeWidth(AXIS_STROKE_WIDTH);
        fieldOverlay.strokeLine(0, 0, 0, length);
    }

    private static void drawRobotInternal(Canvas fieldOverlay, Pose pose, double width, double length, String bodyColor, String headingColor, CoordinateConvention convention) {
        applyFieldTransform(fieldOverlay, convention);

        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

        // Draw custom robot PNG image if enabled
        if (USE_ROBOT_IMAGE && ROBOT_IMAGE_PATH != null && !ROBOT_IMAGE_PATH.isEmpty()) {
            double imgW = length;
            double imgH = width;
            double anchorX = imgW / 2.0;
            double anchorY = imgH / 2.0;
            double imgRotation = heading + Math.toRadians(ROBOT_IMAGE_OFFSET_ROTATION_DEG);

            fieldOverlay.drawImage(
                    ROBOT_IMAGE_PATH,
                    x, y,
                    imgW, imgH,
                    -imgRotation,
                    anchorX, anchorY,
                    false
            );
        }

        // Draw robot polygon outline & heading vector line if enabled
        if (!USE_ROBOT_IMAGE || DRAW_ROBOT_OUTLINE) {
            double halfWidth = width / 2.0;
            double halfLength = length / 2.0;

            double[] xCorners = {halfLength, halfLength, -halfLength, -halfLength};
            double[] yCorners = {halfWidth, -halfWidth, -halfWidth, halfWidth};

            double[] xRotated = new double[4];
            double[] yRotated = new double[4];

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            for (int i = 0; i < 4; i++) {
                xRotated[i] = x + (xCorners[i] * cos - yCorners[i] * sin);
                yRotated[i] = y + (xCorners[i] * sin + yCorners[i] * cos);
            }

            fieldOverlay.setStroke(bodyColor);
            fieldOverlay.setStrokeWidth(ROBOT_OUTLINE_STROKE_WIDTH);
            fieldOverlay.strokePolygon(xRotated, yRotated);

            double headX = x + (halfLength + 6.0) * cos;
            double headY = y + (halfLength + 6.0) * sin;

            fieldOverlay.setStroke(headingColor);
            fieldOverlay.setStrokeWidth(ROBOT_HEADING_STROKE_WIDTH);
            fieldOverlay.strokeLine(x, y, headX, headY);
        }
    }
}
