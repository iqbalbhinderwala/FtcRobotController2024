package org.firstinspires.ftc.teamcode.John;

/**
 * John's physical layout in the robot frame, the same frame Pedro and the Pinpoint use: +X is the
 * drive's forward (axial) direction, +Y is the robot's left, angles are degrees counter-clockwise
 * from +X.
 *
 * John is symmetric front to back (an intake at each axial end), so which end is +X is a convention:
 * it is the end that puts the shooter on the robot's LEFT, so SHOOTER_YAW_DEG is +90.
 *
 * People name the sides from the shooter (standing behind it, looking where it fires), not from
 * Pedro's front: the SHOOTER side, INTAKE A on its left, INTAKE B on its right, and the RAMP side
 * opposite the shooter (where balls wait on an inclined ramp behind the gate / indexer). Pedro's front (+X) is the intake B end, Pedro's back (-X) the intake A end.
 */
public final class RobotGeometry {

    /** The shooter fires out of the left side. */
    public static final double SHOOTER_YAW_DEG = 90.0;
    /** Intake A: left of the shooter (Pedro's back end). */
    public static final double INTAKE_A_YAW_DEG = 180.0;
    /** Intake B: right of the shooter (Pedro's front end). */
    public static final double INTAKE_B_YAW_DEG = 0.0;
    /** Ramp side: opposite the shooter, where balls wait on the ramp behind the gate / indexer. */
    public static final double RAMP_SIDE_YAW_DEG = -90.0;

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
