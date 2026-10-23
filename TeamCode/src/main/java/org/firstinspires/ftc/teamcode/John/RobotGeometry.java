package org.firstinspires.ftc.teamcode.John;

/**
 * John's physical layout in the robot frame, the same frame Pedro and the Pinpoint use: +X is the
 * drive's forward (axial) direction, +Y is the robot's left, angles are degrees counter-clockwise
 * from +X.
 *
 * John is symmetric front to back (an intake at each axial end), so which end is +X is a convention:
 * it is the end that puts the shooter on the robot's LEFT, so SHOOTER_YAW_DEG is +90.
 */
public final class RobotGeometry {

    /** The shooter fires out of the left side. */
    public static final double SHOOTER_YAW_DEG = 90.0;
    /** Intakes at both axial ends, collecting toward the middle. */
    public static final double FRONT_INTAKE_YAW_DEG = 0.0;
    public static final double BACK_INTAKE_YAW_DEG = 180.0;

    /** Frame size, inches: LENGTH along X (axial), WIDTH along Y (lateral). */
    public static final double LENGTH_IN = 17.0;
    public static final double WIDTH_IN = 17.25;

    /** Robot heading (radians, field frame) that points the shooter along a field bearing (radians). */
    public static double headingToShoot(double bearingRad) {
        return bearingRad - Math.toRadians(SHOOTER_YAW_DEG);
    }

    /** Field direction (radians) the shooter points along, for a robot heading (radians). */
    public static double shooterDirection(double robotHeadingRad) {
        return robotHeadingRad + Math.toRadians(SHOOTER_YAW_DEG);
    }

    private RobotGeometry() {}
}
