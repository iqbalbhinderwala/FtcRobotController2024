package org.firstinspires.ftc.teamcode.Vex.Hardware;

/**
 * A simple odometry class that tracks the robot's position.
 * It calculates the new field-centric position based on odometer deltas and
 * a heading value that is provided with every update.
 *
 * This implementation assumes that the heading is externally managed (e.g., by an IMU)
 * and that heading changes between updates are small enough to be approximated as linear.
 *
 * Robot axes convention used:
 *  +Y is forward
 *  +X is to the right (strafe)
 *  Heading is measured in degrees.
 */
public class LinearOdometry {

    public double x, y;
    private double prevLeftOdometer, prevRightOdometer, prevHorizontalOdometer;

    /**
     * Constructs a LinearOdometry object and initializes the robot's pose
     * to the origin (0, 0).
     */
    public LinearOdometry() {
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
     * This method calculates the change in odometer positions, rotates it based on the
     * provided heading, and adds it to the current field position.
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
        // The forward movement (robot's local Y) is the average of the two axial odometers
        double localDeltaY = (deltaLeft + deltaRight) / 2.0;
        // The sideways movement (robot's local X) is from the horizontal odometer
        double localDeltaX = deltaHorizontal;

        // --- Rotate the local changes into the global field coordinate frame ---
        // Convert the current heading to radians for Math functions
        double headingRad = Math.toRadians(heading);
        double sinHeading = Math.sin(headingRad);
        double cosHeading = Math.cos(headingRad);

        // The change in the field's X coordinate is a combination of local forward and strafe movements
        double globalDeltaX = localDeltaX * cosHeading - localDeltaY * sinHeading;
        // The change in the field's Y coordinate is also a combination of both local movements
        double globalDeltaY = localDeltaX * sinHeading + localDeltaY * cosHeading;

        // Update the global pose by adding the calculated displacement.
        this.x += globalDeltaX;
        this.y += globalDeltaY;

        // Store the current odometer readings for the next update cycle
        this.prevLeftOdometer = leftOdometerPos;
        this.prevRightOdometer = rightOdometerPos;
        this.prevHorizontalOdometer = horizontalOdometerPos;
    }
}
