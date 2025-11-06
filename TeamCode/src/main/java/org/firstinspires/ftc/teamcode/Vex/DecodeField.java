package org.firstinspires.ftc.teamcode.Vex;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
}
