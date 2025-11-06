package org.firstinspires.ftc.teamcode.Vex.Hardware;

/**
 * An odometry class that tracks the robot's position, accounting for rotational "twist".
 * It calculates the new field-centric position by modeling the robot's movement as an arc.
 * This provides a more accurate position estimate than linear approximation, especially
 * during turns.
 *
 * This implementation assumes:
 *  - Heading is externally managed (e.g., by an IMU).
 *  - The robot has three odometry pods: two parallel (left/right) and one perpendicular (horizontal).
 *
 * Robot axes convention used:
 *  +Y is forward (front of the robot)
 *  +X is to the right (strafe)
 *  Heading is measured in degrees.
 */
public class TwistOdometry {

    // Robot physical constants
    private final double trackWidth;
    private final double centerWheelOffset;

    // Pose variables
    public double x, y;
    private double prevLeftOdometer, prevRightOdometer, prevHorizontalOdometer;

    /**
     * Constructs a TwistOdometry object with the robot's specific physical dimensions.
     * @param trackWidth The distance between the left and right odometry pods (in inches).
     * @param centerWheelOffset The physical Y-coordinate of the horizontal pod, where the
     *                          axle of the parallel pods is at Y=0 and the front of the
     *                          robot is in the +Y direction.
     */
    public TwistOdometry(double trackWidth, double centerWheelOffset) {
        this.trackWidth = trackWidth;
        this.centerWheelOffset = centerWheelOffset;
        this.x = 0;
        this.y = 0;
        this.prevLeftOdometer = 0;
        this.prevRightOdometer = 0;
        this.prevHorizontalOdometer = 0;
    }

    /**
     * Sets or resets the robot's position on the field.
     * This method is essential for setting the starting position.
     * It also resets the previous odometer readings to the current hardware state.
     * @param startX The starting X position of the robot.
     * @param startY The starting Y position of the robot.
     * @param leftOdometerPos The current position of the left axial odometer.
     * @param rightOdometerPos The current position of the right axial odometer.
     * @param horizontalOdometerPos The current position of the horizontal/lateral odometer.
     */
    public void initPose(double startX, double startY,
                         double leftOdometerPos, double rightOdometerPos, double horizontalOdometerPos) {
        this.x = startX;
        this.y = startY;
        this.prevLeftOdometer = leftOdometerPos;
        this.prevRightOdometer = rightOdometerPos;
        this.prevHorizontalOdometer = horizontalOdometerPos;
    }

    /**
     * Updates the robot's pose based on the latest odometer readings and current heading.
     * This method models the robot's movement as an arc to calculate the change in
     * field position.
     *
     * @param heading The robot's current heading in degrees.
     * @param leftOdometerPos The new (current) position of the left axial odometer.
     * @param rightOdometerPos The new (current) position of the right axial odometer.
     * @param horizontalOdometerPos The new (current) position of the horizontal/lateral odometer.
     */
    public void update(double heading,
                       double leftOdometerPos, double rightOdometerPos, double horizontalOdometerPos) {
        // Calculate the change in odometer readings since the last update
        double deltaLeft = leftOdometerPos - prevLeftOdometer;
        double deltaRight = rightOdometerPos - prevRightOdometer;
        double deltaHorizontal = horizontalOdometerPos - prevHorizontalOdometer;

        // --- Calculate change in robot's local coordinates ---
        // Change in orientation (theta) is calculated from the difference in the parallel odometers.
        // A counter-clockwise turn yields a positive deltaAngle.
        double deltaAngleRad = (deltaRight - deltaLeft) / trackWidth;

        // Forward movement (local Y) is the average of the two parallel odometers
        double localDeltaY = (deltaLeft + deltaRight) / 2.0;

        // Sideways movement (local X) is from the horizontal odometer, corrected for rotation.
        // The rotation of the robot induces movement in the horizontal pod (slip), which is not true strafing.
        // We must add the calculated slip to the measured delta to get the actual localDeltaX.
        // Note the PLUS sign, which allows centerWheelOffset to match the physical Y-coordinate given a CCW turn.
        double localDeltaX = deltaHorizontal + (centerWheelOffset * deltaAngleRad);

        // --- Accurately model the robot's motion as an arc to "untwist" the local deltas ---
        // This calculates the equivalent straight-line displacement vector relative to the robot's
        // orientation at the START of the update cycle. If the robot turned, we update
        // localDeltaX/Y to be the chord of the arc traveled.
        boolean correct_twist = false;
        if (correct_twist && Math.abs(deltaAngleRad) > 1e-6) { // Check for non-zero rotation (curved path)
            // If the robot is turning, we model the path as an arc.
            // These formulas use trigonometric identities to find the endpoint of the arc.
            double sinDeltaAngle = Math.sin(deltaAngleRad);
            double cosDeltaAngle = Math.cos(deltaAngleRad);

            // The terms (sin(d_theta)/d_theta) and ((1-cos(d_theta))/d_theta) can be thought of as
            // averaging factors that account for the continuous change in heading over the arc.
            double sineTerm = sinDeltaAngle / deltaAngleRad;
            double cosTerm = (1.0 - cosDeltaAngle) / deltaAngleRad;

            double twistedLocalDeltaX = localDeltaX;
            double twistedLocalDeltaY = localDeltaY;
            // Update localDeltaX to the "untwisted" value
            localDeltaX = (twistedLocalDeltaX * sineTerm) - (twistedLocalDeltaY * cosTerm);
            // Update localDeltaY to the "untwisted" value
            localDeltaY = (twistedLocalDeltaX * cosTerm) + (twistedLocalDeltaY * sineTerm);
        }
        // If the robot did not turn, localDeltaX and localDeltaY are already the correct untwisted values.


        // --- Rotate the untwisted local displacement into the global field coordinate frame ---
        // We use the robot's absolute heading (from the IMU) to perform this rotation.
        double headingRad = Math.toRadians(heading);
        double sinHeading = Math.sin(headingRad);
        double cosHeading = Math.cos(headingRad);

        double globalDeltaX = localDeltaX * cosHeading - localDeltaY * sinHeading;
        double globalDeltaY = localDeltaX * sinHeading + localDeltaY * cosHeading;

        // Update the global pose by adding the final calculated displacement.
        this.x += globalDeltaX;
        this.y += globalDeltaY;

        // Store the current odometer readings for the next update cycle
        this.prevLeftOdometer = leftOdometerPos;
        this.prevRightOdometer = rightOdometerPos;
        this.prevHorizontalOdometer = horizontalOdometerPos;
    }
}
