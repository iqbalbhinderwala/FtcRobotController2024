package org.firstinspires.ftc.teamcode.Vex;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A class dedicated to providing the robot's heading from an IMU,
 * while allowing the zero-point to be set to any arbitrary heading.
 */
public class IMUHeadingProvider {

    private final IMU imu;
    // Stores the desired heading value that corresponds to the IMU's hardware zero point.
    private double referenceHeading = 0;

    /**
     * Constructor that requires a hardware-initialized IMU object.
     * @param imu The instance of the IMU to use for heading data.
     */
    public IMUHeadingProvider(IMU imu) {
        this.imu = imu;
    }

    /**
     * Gets the robot's current heading in degrees.
     * The returned value is normalized to the range [-180, 180).
     * @return The adjusted heading in degrees.
     */
    public double getHeading() {
        // Get the raw yaw relative to the last hardware reset.
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Add the baseline to the raw value.
        double adjustedYaw = rawYaw + referenceHeading;

        // Normalize the angle to be within the range [-180, 180).
        while (adjustedYaw >= 180) adjustedYaw -= 360;
        while (adjustedYaw < -180) adjustedYaw += 360;

        return adjustedYaw;
    }

    /**
     * Resets the underlying IMU and sets the robot's heading to a new value.
     * For example, calling resetYaw(90) makes the robot's current direction read as 90 degrees.
     *
     * @param newHeading The desired heading (in degrees) for the robot's current orientation.
     */
    public void resetYaw(double newHeading) {
        // 1. Reset the IMU's underlying zero reference to this exact moment.
        imu.resetYaw();

        // 2. Get the residual raw heading after the reset. This may not be exactly zero.
        double offset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // 3. Set the reference heading to the new desired heading, adjusted for the offset.
        this.referenceHeading = newHeading - offset;
    }
}
