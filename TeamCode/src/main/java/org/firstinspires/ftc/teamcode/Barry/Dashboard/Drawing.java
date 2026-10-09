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
        FTC,    // Center origin (0,0), +X UP, +Y LEFT
        PEDRO   // Bottom-left origin (0,0), +X RIGHT, +Y UP (0 to 144)
    }

    // Configurable Dashboard Settings
    public static CoordinateConvention CONVENTION = CoordinateConvention.PEDRO;

    public static boolean DRAW_FIELD = true;
    public static boolean DRAW_AXES = true;
    public static boolean DRAW_GRID_POINTS = true;
    public static boolean USE_CUSTOM_IMAGE = true;

    public static String FIELD_IMAGE_PATH = "/images/BIOBUZZ_field_2026.webp";
    public static double FIELD_IMAGE_WIDTH = 144.0;
    public static double FIELD_IMAGE_HEIGHT = 144.0;

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

    // =========================================================================
    // Internal Drawing Helpers
    // =========================================================================

    private static void applyFieldTransform(Canvas fieldOverlay, CoordinateConvention convention) {
        if (convention == CoordinateConvention.PEDRO) {
            fieldOverlay
                    .setRotation(Math.toRadians(-90.0))
                    .setTranslation(-72.0, 72.0);
        } else {
            fieldOverlay
                    .setRotation(0.0)
                    .setTranslation(0.0, 0.0);
        }
    }

    private static void drawFieldInternal(Canvas fieldOverlay, CoordinateConvention convention) {
        applyFieldTransform(fieldOverlay, convention);

        if (convention == CoordinateConvention.PEDRO) {
            fieldOverlay.drawImage(
                    FIELD_IMAGE_PATH,
                    0.0, 144.0,
                    FIELD_IMAGE_WIDTH, FIELD_IMAGE_HEIGHT,
                    0.0, 0, 0, false
            );
        } else {
            fieldOverlay.drawImage(
                    FIELD_IMAGE_PATH,
                    -72.0, 72.0,
                    FIELD_IMAGE_WIDTH, FIELD_IMAGE_HEIGHT,
                    0.0, 0, 0, false
            );
        }
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
                            .fillText(label, x + 1.5, y + 1.5, GRID_TEXT_FONT, Math.toRadians(GRID_TEXT_ROTATION_DEG), false);
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
