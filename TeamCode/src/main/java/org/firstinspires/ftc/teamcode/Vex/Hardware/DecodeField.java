package org.firstinspires.ftc.teamcode.Vex.Hardware;

import android.util.Log;
import java.util.List;

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
    private static final double ROBOT_LENGTH = 17.5; // inches
    private static final double ROBOT_WIDTH  = 17.0; // inches
    private static final double ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2.0; // inches
    private static final double ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0; // inches
    private static final double SHOOTER_OFFSET_INCHES = 4.0; // inches to the right

    // Alliance corner coordinates (in inches).
    // It's assumed that the odometry system uses inches as its unit.
    public static final double RED_CORNER_X = -3.0 * TILE; // -72 inches
    public static final double RED_CORNER_Y = 3.0 * TILE;  // +72 inches
    public static final double BLUE_CORNER_X = -3.0 * TILE; // -72 inches
    public static final double BLUE_CORNER_Y = -3.0 * TILE; // -72 inches

    public static final double MINIMUM_SHOOTING_DISTANCE = 2.4 * TILE;
    public static final double MAXIMUM_NEAR_SHOOTING_ANGLE_ERROR_DEGREES = 7.0; // degrees
    public static final double MAXIMUM_FAR_SHOOTING_ANGLE_ERROR_DEGREES = 3.5; // degrees

    public enum Alliance { RED, BLUE, UNKNOWN }

    // --- ENUM FOR KEY LOCATIONS ON THE FIELD ---
    // This enum uses Pose2d to store the location and heading.
    // See also org/firstinspires/ftc/vision/apriltag/AprilTagGameDatabase.java
    public enum KeyLocation {
        UNKNOWN(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90)),
        // Red Alliance Locations

        // Red Alliance, Audience Wall, 0.5 tile off center line, facing North
        RED__AUDIENCE_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                3 * TILE - ROBOT_HALF_LENGTH,
                0.5 * TILE,  // Right side
                AngleUnit.DEGREES, 90)), // North facing

        // Blue Alliance, Audience Wall, 0.5 tile off center line, facing North
        BLUE__AUDIENCE_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                3 * TILE - ROBOT_HALF_LENGTH,
                -0.5 * TILE,  // Left side
                AngleUnit.DEGREES, 90)), // North facing

        // Red Alliance, Obelisk Wall, 0.5 tile off center line, facing North
        RED__OBELISK_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                -3 * TILE + ROBOT_HALF_LENGTH,
                2 * TILE - ROBOT_HALF_LENGTH,  // Right side
                AngleUnit.DEGREES, 90)), // North facing

        // Blue Alliance, Obelisk Wall, 0.5 tile off center line, facing North
        BLUE__OBELISK_WALL__HEADING_NORTH(new Pose2D(DistanceUnit.INCH,
                -3 * TILE + ROBOT_HALF_LENGTH,
                -2 * TILE + ROBOT_HALF_LENGTH,  // Left side
                AngleUnit.DEGREES, 90)), // North facing

        // Red Alliance, Facing Red Target Corner
        RED__START_NEAR__FACING_TARGET(new Pose2D(DistanceUnit.INCH,
                -2.493 * TILE,
                +1.918 * TILE,
                AngleUnit.DEGREES, 36)),

        RED__SHOOT_NEAR(new Pose2D(DistanceUnit.INCH,
                -1.5 * TILE,
                +0.7 * TILE,
                AngleUnit.DEGREES, 36)),

        // Blue Alliance, Facing Red Target Corner
        BLUE__START_NEAR__FACING_TARGET(new Pose2D(DistanceUnit.INCH,
                -2.493 * TILE,
                -1.918 * TILE,
                AngleUnit.DEGREES, 180-36)),

        BLUE__SHOOT_NEAR(new Pose2D(DistanceUnit.INCH,
                -1.3 * TILE,
                -0.9 * TILE,
                AngleUnit.DEGREES, 180-36)),


        // Added a semicolon here to terminate the list of enum constants.
        ;

        private final Pose2D pose;

        KeyLocation(Pose2D pose) {
            this.pose = pose;
        }

        public Pose2D getPose() { return pose; }

        public static List<KeyLocation> getStartingLocations(Alliance currentAlliance) {
            switch (currentAlliance) {
                case RED:
                    return List.of(RED__START_NEAR__FACING_TARGET, RED__AUDIENCE_WALL__HEADING_NORTH, RED__OBELISK_WALL__HEADING_NORTH);
                case BLUE:
                    return List.of(BLUE__START_NEAR__FACING_TARGET, BLUE__AUDIENCE_WALL__HEADING_NORTH, BLUE__OBELISK_WALL__HEADING_NORTH);
                default:
                    return List.of();
            }
        }
    }

    private static Pose2D getShooterPose(Pose2D robotPose) {
        double currentX = robotPose.getX(DistanceUnit.INCH);
        double currentY = robotPose.getY(DistanceUnit.INCH);
        double currentHeading = robotPose.getHeading(AngleUnit.RADIANS);

        // Calculate the shooter's position, accounting for the offset to the right.
        // Positive heading is CCW from the positive Y axis. The "right" vector is (cos(h), sin(h)).
        double shooterX = currentX + SHOOTER_OFFSET_INCHES * Math.cos(currentHeading);
        double shooterY = currentY + SHOOTER_OFFSET_INCHES * Math.sin(currentHeading);

        return new Pose2D(DistanceUnit.INCH, shooterX, shooterY, AngleUnit.RADIANS, currentHeading);
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

        // 2. Get the shooter's position from the new method
        Pose2D shooterPose = getShooterPose(robotPose);
        double shooterX = shooterPose.getX(DistanceUnit.INCH);
        double shooterY = shooterPose.getY(DistanceUnit.INCH);

        // 3. Calculate the distance using the distance formula from the shooter.
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
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

        // 2. Get the robot's current position and heading from the pose.
        double currentX = robotPose.getX(DistanceUnit.INCH);
        double currentY = robotPose.getY(DistanceUnit.INCH);
        double currentHeading = robotPose.getHeading(AngleUnit.RADIANS);

        // 2a. Get the shooter's position from the new method.
        Pose2D shooterPose = getShooterPose(robotPose);
        double shooterX = shooterPose.getX(DistanceUnit.INCH);
        double shooterY = shooterPose.getY(DistanceUnit.INCH);

        // 3. Calculate the vector from the shooter to the corner.
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;

        // Log initial data
        Log.d(TAG, "getTurnAngle_v1: Alliance=" + currentAlliance);
        Log.d(TAG, String.format("getTurnAngle_v1: Robot Pose (X, Y, HeadingDeg): (%.2f, %.2f, %.2f)",
                currentX, currentY, robotPose.getHeading(AngleUnit.DEGREES)));
        Log.d(TAG, String.format("getTurnAngle_v1: Shooter Pose (X, Y): (%.2f, %.2f)", shooterX, shooterY));
        Log.d(TAG, String.format("getTurnAngle_v1: Target Corner (X, Y): (%.2f, %.2f)", targetX, targetY));
        Log.d(TAG, String.format("getTurnAngle_v1: Vector to Target (dx, dy): (%.2f, %.2f)", dx, dy));


        // 4. Calculate the absolute angle of the target vector relative to the positive Y-axis.
        //    Math.atan2(-x, y) gives the angle from the positive Y-axis to the vector (x, y).
        double absoluteAngleToTarget = Math.atan2(-dx, dy);

        // 5. Get the robot's current heading (already relative to the positive Y-axis).
        //double currentHeading = robotPose.getHeading(AngleUnit.RADIANS);

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
     * Checks if the robot is within the valid shooting range and orientation.
     * @param currentAlliance The current alliance.
     * @param robotPose The robot's current pose.
     * @return True if the robot is far enough and oriented correctly for shooting, false otherwise.
     */
    public static boolean isInRangeForShooting(Alliance currentAlliance, Pose2D robotPose) {
        double distance = getDistanceToAllianceCorner(currentAlliance, robotPose);
        double turnAngle = getTurnAngleToAllianceCorner(currentAlliance, robotPose);
        boolean isFar = (distance > 5 * TILE);
        double maximum_shooting_angle_error = (isFar ?
                MAXIMUM_FAR_SHOOTING_ANGLE_ERROR_DEGREES :
                MAXIMUM_NEAR_SHOOTING_ANGLE_ERROR_DEGREES);
        return distance >= MINIMUM_SHOOTING_DISTANCE && Math.abs(turnAngle) <= maximum_shooting_angle_error;
    }

    private static final String TAG = "VEX::DecodeField";
}
