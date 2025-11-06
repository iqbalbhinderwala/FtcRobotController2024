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
    public static final double RED_CORNER_X = -3.0 * 24.0; // -72 inches
    public static final double RED_CORNER_Y = 3.0 * 24.0;  // +72 inches
    public static final double BLUE_CORNER_X = -3.0 * 24.0; // -72 inches
    public static final double BLUE_CORNER_Y = -3.0 * 24.0; // -72 inches

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
     * Calculates the angle from the robot to the current alliance corner.
     * @param currentAlliance The current alliance.
     * @param robotPose The robot's current pose.
     * @return The angle in degrees.
     */
    public static double getTurnAngleToAllianceCorner(Alliance currentAlliance, Pose3D robotPose) {
        // 1. Get the target coordinates based on the current alliance.
        double targetX = (currentAlliance == Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
        double targetY = (currentAlliance == Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

        // 2. Get the robot's current position from the pose.
        double currentX = robotPose.getPosition().x;
        double currentY = robotPose.getPosition().y;

        // 3. Calculate the angle to the target.
        return Math.toDegrees(Math.atan2(targetY - currentY, targetX - currentX));
    }
}
