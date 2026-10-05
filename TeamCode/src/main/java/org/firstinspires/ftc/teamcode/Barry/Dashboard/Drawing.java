
package org.firstinspires.ftc.teamcode.Barry.Dashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;

/**
 * Utility class for rendering robot positions and graphics on the FTC Dashboard Field Overlay.
 */
public class Drawing {

    public static double ROBOT_WIDTH = 18.0;
    public static double ROBOT_LENGTH = 18.0;

    /**
     * Draws the robot's bounding box and heading line on the FTC Dashboard Canvas.
     *
     * @param fieldOverlay The FTC Dashboard Canvas
     * @param pose         The robot's current pose (x, y, heading in radians)
     */
    public static void drawRobot(Canvas fieldOverlay, Pose pose) {
        drawRobot(fieldOverlay, pose, ROBOT_WIDTH, ROBOT_LENGTH, "blue", "red");
    }

    /**
     * Draws the robot with custom dimensions and colors on the FTC Dashboard Canvas.
     *
     * @param fieldOverlay The FTC Dashboard Canvas
     * @param pose         The robot's current pose
     * @param width        Robot width in inches
     * @param length       Robot length in inches
     * @param bodyColor    Color for the robot outline
     * @param headingColor Color for the heading vector line
     */
    public static void drawRobot(Canvas fieldOverlay, Pose pose, double width, double length, String bodyColor, String headingColor) {
        if (fieldOverlay == null || pose == null) {
            return;
        }

        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

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

        // Draw robot body outline
        fieldOverlay.setStroke(bodyColor);
        fieldOverlay.setStrokeWidth(2);
        fieldOverlay.strokePolygon(xRotated, yRotated);

        // Draw heading line
        double headX = x + (halfLength + 6.0) * cos;
        double headY = y + (halfLength + 6.0) * sin;

        fieldOverlay.setStroke(headingColor);
        fieldOverlay.setStrokeWidth(3);
        fieldOverlay.strokeLine(x, y, headX, headY);
    }
}
