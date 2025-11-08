package org.firstinspires.ftc.teamcode.Vex.Hardware;

import android.util.Log;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * A utility class for decoding field positions and orientations for the VEX Robotics Competition.
 * This class provides enums and constants to represent team colors, starting walls, heading directions,
 * and key locations on the field with their corresponding poses.
 *
 * The field is divided into tiles, and robot dimensions are defined for accurate positioning.
 */
public class DecodeField {

    // --- FIELD & ROBOT CONSTANTS ---
    // Moved these declarations to the top so they are defined before being used in the enum.
    private static final double TILE = 24.0; // inches
    private static final double ROBOT_LENGTH = 18.0; // inches
    private static final double ROBOT_WIDTH = 16.0; // inches
    private static final double ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2.0; // inches
    private static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0; // inches

    // Alliance corner coordinates (in inches).
    // It's assumed that the odometry system uses inches as its unit.
    public static final double RED_CORNER_X = -3.0 * TILE; // -72 inches
    public static final double RED_CORNER_Y = 3.0 * TILE;  // +72 inches
    public static final double BLUE_CORNER_X = -3.0 * TILE; // -72 inches
    public static final double BLUE_CORNER_Y = -3.0 * TILE; // -72 inches

    public static final double MINIMUM_SHOOTING_DISTANCE = 2 * TILE * Math.sqrt(2);

    public enum Alliance { RED, BLUE, UNKNOWN }

    // --- ENUM FOR KEY LOCATIONS ON THE FIELD ---
    // This enum uses Pose2d to store the location and heading.
    // See also org/firstinspires/ftc/vision/apriltag/AprilTagGameDatabase.java
    public enum KeyLocation {
        UNKNOWN(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90)),
        // Red Alliance Locations

        // Red Alliance, Audience Wall, 1 tile off center line, facing North
        RED__AUDIENCE_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                3 * TILE - ROBOT_HALF_LENGTH,
                1 * TILE,  // Right side
                AngleUnit.DEGREES, 90)), // North facing

        // Blue Alliance, Audience Wall, 1 tile off center line, facing North
        BLUE__AUDIENCE_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                3 * TILE - ROBOT_HALF_LENGTH,
                -1 * TILE,  // Left side
                AngleUnit.DEGREES, 90)), // North facing

        // Red Alliance, Obelisk Wall, 1 tile off center line, facing North
        RED__OBELISK_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                -3 * TILE + ROBOT_HALF_LENGTH,
                1 * TILE,  // Right side
                AngleUnit.DEGREES, 90)), // North facing

        // Blue Alliance, Obelisk Wall, 1 tile off center line, facing North
        BLUE__OBELISK_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                -3 * TILE + ROBOT_HALF_LENGTH,
                -1 * TILE,  // Left side
                AngleUnit.DEGREES, 90)), // North facing

        // Added a semicolon here to terminate the list of enum constants.
        ;

        private final Pose2D pose;

        KeyLocation(Pose2D pose) {
            this.pose = pose;
        }

        public Pose2D getPose() { return pose; }
    }

    /**
     * Calculates the distance from the robot to the current alliance corner.
     * @param currentAlliance The current alliance.
     * @param robotPose The robot's current pose.
     * @return The distance in inches.
     */
    public static double getDistanceToAllianceCorner(Alliance currentAlliance, Pose2D robotPose) {
        // 1. Get the target coordinates based on the current alliance.
        double targetX = (currentAlliance == Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
        double targetY = (currentAlliance == Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

        // 2. Get the robot's current position from the pose.
        double currentX = robotPose.getX(DistanceUnit.INCH);
        double currentY = robotPose.getY(DistanceUnit.INCH);

        // 3. Calculate the distance using the distance formula.
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        return Math.sqrt(dx*dx + dy*dy);
    }

    /**
     * Calculates the angle the robot needs to turn to face the current alliance corner.
     * Assumes a coordinate system where 0 heading is along the positive Y-axis.
     *
     * @param currentAlliance The current alliance (RED or BLUE).
     * @param robotPose The robot's current 2D pose from VexOdometryDriveTrain.getPose2D().
     * @return The required turn angle in degrees, in the range [-180, 180].
     *         A positive value indicates a counter-clockwise (left) turn, and a negative value
     *         indicates a clockwise (right) turn.
     */
    public static double getTurnAngleToAllianceCorner(Alliance currentAlliance, Pose2D robotPose) {
        // 1. Define the target coordinates for the alliance corner.
        double targetX = (currentAlliance == Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
        double targetY = (currentAlliance == Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

        // 2. Get the robot's current position from the pose.
        double currentX = robotPose.getX(DistanceUnit.INCH);
        double currentY = robotPose.getY(DistanceUnit.INCH);

        // 3. Calculate the vector from the robot to the corner.
        double dx = targetX - currentX;
        double dy = targetY - currentY;

        // Log initial data
        Log.d(TAG, "getTurnAngle_v1: Alliance=" + currentAlliance);
        Log.d(TAG, String.format("getTurnAngle_v1: Robot Pose (X, Y, HeadingDeg): (%.2f, %.2f, %.2f)",
                currentX, currentY, robotPose.getHeading(AngleUnit.DEGREES)));
        Log.d(TAG, String.format("getTurnAngle_v1: Target Corner (X, Y): (%.2f, %.2f)", targetX, targetY));
        Log.d(TAG, String.format("getTurnAngle_v1: Vector to Target (dx, dy): (%.2f, %.2f)", dx, dy));


        // 4. Calculate the absolute angle of the target vector relative to the positive Y-axis.
        //    Math.atan2(x, y) gives the angle from the positive Y-axis to the vector (x, y).
        double absoluteAngleToTarget = Math.atan2(-dx, dy);

        // 5. Get the robot's current heading (already relative to the positive Y-axis).
        double currentHeading = robotPose.getHeading(AngleUnit.RADIANS);

        // 6. The required turn is the difference between where we want to point and where we are pointing.
        double turnAngle = absoluteAngleToTarget - currentHeading;

        Log.d(TAG, String.format("getTurnAngle_v1: Absolute Angle to Target (deg): %.2f", Math.toDegrees(absoluteAngleToTarget)));
        Log.d(TAG, String.format("getTurnAngle_v1: Current Heading (deg): %.2f", Math.toDegrees(currentHeading)));
        Log.d(TAG, String.format("getTurnAngle_v1: Initial Turn Angle (deg): %.2f", Math.toDegrees(turnAngle)));

        // 7. Normalize the angle to the range [-PI, PI] to ensure the shortest turn.
        while (turnAngle > Math.PI) {
            turnAngle -= 2 * Math.PI;
        }
        while (turnAngle <= -Math.PI) {
            turnAngle += 2 * Math.PI;
        }

        double finalAngleDegrees = Math.toDegrees(turnAngle);
        Log.d(TAG, String.format("getTurnAngle_v1: Normalized Turn Angle (deg): %.2f", finalAngleDegrees));

        // 8. Convert the result from radians to degrees for the return value.
        return finalAngleDegrees;
    }

    /**
     * Checks if the robot is within the valid shooting range.
     * @param currentAlliance The current alliance.
     * @param robotPose The robot's current pose.
     * @return True if the robot is far enough to shoot, false otherwise.
     */
    public static boolean isInRangeForShooting(Alliance currentAlliance, Pose2D robotPose) {
        return getDistanceToAllianceCorner(currentAlliance, robotPose) >= MINIMUM_SHOOTING_DISTANCE;
    }

    private static final String TAG = "VEX::DecodeField";
}
