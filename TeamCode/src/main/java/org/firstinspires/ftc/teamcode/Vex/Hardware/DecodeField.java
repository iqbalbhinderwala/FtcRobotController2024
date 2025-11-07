package org.firstinspires.ftc.teamcode.Vex.Hardware;

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
    public static double getDistanceToAllianceCorner(Alliance currentAlliance, Pose3D robotPose) {
        // 1. Get the target coordinates based on the current alliance.
        double targetX = (currentAlliance == Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
        double targetY = (currentAlliance == Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

        // 2. Get the robot's current position from the pose.
        double currentX = robotPose.getPosition().x;
        double currentY = robotPose.getPosition().y;

        // 3. Calculate the distance using the distance formula.
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        return Math.sqrt(dx*dx + dy*dy);
    }

    /**
     * Calculates the angle the robot needs to turn to face the current alliance corner.
     * It computes the signed angle between the robot's forward vector and the target vector
     * using the atan2(cross_product, dot_product) identity.
     *
     * @param currentAlliance The current alliance (RED or BLUE).
     * @param robotPose The robot's current 2D pose from VexOdometryDriveTrain.getPose2D().
     * @return The required turn angle in degrees, in the range [-180, 180].
     *         A positive value indicates a counter-clockwise (left) turn, and a negative value
     *         indicates a clockwise (right) turn.
     */
    public static double getTurnAngleToAllianceCorner_v1(Alliance currentAlliance, Pose2D robotPose) {
        // 1. Define the target coordinates for the alliance corner.
        double targetX = (currentAlliance == Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
        double targetY = (currentAlliance == Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

        // 2. Get the robot's current position from the pose, specifying INCH units.
        double currentX = robotPose.getX(DistanceUnit.INCH);
        double currentY = robotPose.getY(DistanceUnit.INCH);

        // 3. Compute the 2D robot-to-target vector (T).
        double targetVecX = targetX - currentX;
        double targetVecY = targetY - currentY;

        // 4. Compute the 2D forward direction vector from the robot's current heading (F).
        // Get heading in radians.
        double currentHeadingRadians = robotPose.getHeading(AngleUnit.RADIANS);

        // **CRITICAL CORRECTION**: Adjust for the heading convention where 0 is the positive Y-axis.
        // To use standard math functions (where 0 is +X axis), we must convert our heading
        // by subtracting 90 degrees (PI/2 radians).
        double mathHeading = currentHeadingRadians - (Math.PI / 2.0);
        double forwardVecX = Math.cos(mathHeading);
        double forwardVecY = Math.sin(mathHeading);

        // 5. Calculate the dot product (F · T) and the 2D cross product's Z-component.
        double dotProduct = forwardVecX * targetVecX + forwardVecY * targetVecY;
        double crossProductZ = forwardVecX * targetVecY - forwardVecY * targetVecX;

        // 6. Use atan2(cross, dot) to find the signed angle directly from the forward vector to the target vector.
        double turnAngleRadians = Math.atan2(crossProductZ, dotProduct);

        // 7. Convert the result from radians to degrees for the return value.
        return Math.toDegrees(turnAngleRadians);
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
    public static double getTurnAngleToAllianceCorner_v2(Alliance currentAlliance, Pose2D robotPose) {
        // 1. Define the target coordinates for the alliance corner.
        double targetX = (currentAlliance == Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
        double targetY = (currentAlliance == Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

        // 2. Get the robot's current position from the pose.
        double currentX = robotPose.getX(DistanceUnit.INCH);
        double currentY = robotPose.getY(DistanceUnit.INCH);

        // 3. Calculate the vector from the robot to the corner.
        double dx = targetX - currentX;
        double dy = targetY - currentY;

        // 4. Calculate the absolute angle of the target vector relative to the positive Y-axis.
        //    Math.atan2(x, y) gives the angle from the positive Y-axis to the vector (x, y).
        double absoluteAngleToTarget = Math.atan2(dx, dy);

        // 5. Get the robot's current heading (already relative to the positive Y-axis).
        double currentHeading = robotPose.getHeading(AngleUnit.RADIANS);

        // 6. The required turn is the difference between where we want to point and where we are pointing.
        double turnAngle = absoluteAngleToTarget - currentHeading;

        // 7. Normalize the angle to the range [-PI, PI] to ensure the shortest turn.
        while (turnAngle > Math.PI) {
            turnAngle -= 2 * Math.PI;
        }
        while (turnAngle <= -Math.PI) {
            turnAngle += 2 * Math.PI;
        }

        // 8. Convert the result from radians to degrees for the return value.
        return Math.toDegrees(turnAngle);
    }

    /**
     * Checks if the robot is within the valid shooting range.
     * @param currentAlliance The current alliance.
     * @param robotPose The robot's current pose.
     * @return True if the robot is far enough to shoot, false otherwise.
     */
    public static boolean isInRangeForShooting(Alliance currentAlliance, Pose3D robotPose) {
        return getDistanceToAllianceCorner(currentAlliance, robotPose) >= MINIMUM_SHOOTING_DISTANCE;
    }
}
