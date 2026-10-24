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
 * Pedro's front: the SHOOTER side, INTAKE A on its left, INTAKE B on its right, and the CAMERA wall
 * opposite the shooter (the Limelight is mounted there, and balls wait on an inclined ramp behind
 * the gate / indexer on that side). Pedro's front (+X) is the intake B end, Pedro's back (-X) the
 * intake A end.
 */
public final class RobotGeometry {

    /** The shooter fires out of the left side. */
    public static final double SHOOTER_YAW_DEG = 90.0;
    /** Intake A: left of the shooter (Pedro's back end). */
    public static final double INTAKE_A_YAW_DEG = 180.0;
    /** Intake B: right of the shooter (Pedro's front end). */
    public static final double INTAKE_B_YAW_DEG = 0.0;
    /** Camera wall: opposite the shooter; the camera is mounted there (balls wait on a ramp on that side). */
    public static final double CAMERA_WALL_YAW_DEG = -90.0;

    /** Frame size, inches: LENGTH along X (axial), WIDTH along Y (lateral). */
    public static final double LENGTH_IN = 17.0;
    public static final double WIDTH_IN = 17.25;

    /**
     * Camera (Limelight) lens, robot frame (inches / degrees), measured 2026-10-23: centered between
     * the intakes, 40 mm in from the camera wall, 400 mm above the floor, looking toward the shooter
     * side (across the robot, so aiming the camera aims the shooter) and tilted 39 deg up.
     */
    public static final double CAMERA_FORWARD_IN = 0.0;
    public static final double CAMERA_LEFT_IN = -(WIDTH_IN / 2.0 - 40.0 / 25.4);   // -7.05
    public static final double CAMERA_HEIGHT_IN = 400.0 / 25.4;                    // 15.75
    public static final double CAMERA_YAW_DEG = SHOOTER_YAW_DEG;
    public static final double CAMERA_PITCH_DEG = 39.0;

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
