package org.firstinspires.ftc.teamcode.Vex.Hardware;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// FROM VexIMUOmniDriveTrain.java

public class VexOdometryDriveTrain {

    // Parent OpMode
    private LinearOpMode opMode;

    // Core Components
    private IMUHeadingProvider headingProvider;
    private TwistOdometry odometry;

    // Hardware
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor leftOdometer, rightOdometer, horizontalOdometer;


    /**
     * Constructor for the drivetrain.
     * @param opMode The parent LinearOpMode.
     */
    public VexOdometryDriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;
        this.odometry = new TwistOdometry(ODOMETER_TRACK_WIDTH_INCH, ODOMETER_CENTER_WHEEL_OFFSET_INCH);
    }

    /**
     * Initializes all hardware components (motors, odometers, IMU).
     */
    public void init() {
        initMotors(opMode.hardwareMap);
        initIMU(opMode.hardwareMap);
    }

    /**
     * Initializes all drive and odometry motors.
     * @param hardwareMap The robot's hardware map.
     */
    private void initMotors(HardwareMap hardwareMap) {
        // --- Initialize Drivetrain Motors ---
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "wheel front left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "wheel front right");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "wheel back left");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "wheel back right");

        leftFrontDrive .setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive  .setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive .setDirection(DcMotor.Direction.FORWARD);

        // --- Initialize Odometry Pods ---
        leftOdometer = hardwareMap.get(DcMotor.class, "wheel front left");
        rightOdometer = hardwareMap.get(DcMotor.class, "wheel front right");
        horizontalOdometer = hardwareMap.get(DcMotor.class, "m3");

        leftOdometer.setDirection(DcMotor.Direction.REVERSE); // Y +-ve forward
        rightOdometer.setDirection(DcMotor.Direction.FORWARD); // Y +-ve forward
        horizontalOdometer.setDirection(DcMotor.Direction.FORWARD); // X +-ve right

        // TurnToHeading() relies on encoders being disabled.
        for (DcMotor motor : new DcMotor[]{leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Reset all odometer encoders
        for (DcMotor motor : new DcMotor[]{leftOdometer, rightOdometer, horizontalOdometer}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Initializes the IMU and the heading provider.
     * @param hardwareMap The robot's hardware map.
     */
    private void initIMU(HardwareMap hardwareMap) {
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); // Reset the IMU heading.

        headingProvider = new IMUHeadingProvider(imu);
    }

    /**
     * Gets the current heading of the robot from the IMU.
     * @return The robot's heading in degrees.
     */
    public double getHeading() {
        return headingProvider.getHeading();
    }

    /**
     * Sets the robot's starting position and heading on the field.
     * @param startX Initial field X-coordinate in inches.
     * @param startY Initial field Y-coordinate in inches.
     * @param startHeading Initial field heading in degrees.
     */
    public void resetPose(double startX, double startY, double startHeading) {
        odometry.initPose(startX, startY,
                getLeftOdometerInches(), getRightOdometerInches(), getHorizontalOdometerInches());
        headingProvider.resetYaw(startHeading);
    }

    /**
     * Updates the robot's position on the field without changing the heading.
     * @param x The new field X-coordinate in inches.
     * @param y The new field Y-coordinate in inches.
     */
    public void updatePosition(double x, double y) {
        odometry.initPose(x, y,
                getLeftOdometerInches(), getRightOdometerInches(), getHorizontalOdometerInches());
    }

    /**
     * Updates the odometry with the latest sensor readings.
     * This should be called on each loop iteration.
     */
    public void update() {
        // Update the odometry with the latest sensor readings
        updateOdometry();
    }

    private void updateOdometry() {
        odometry.update(
                headingProvider.getHeading(),
                getLeftOdometerInches(),
                getRightOdometerInches(),
                getHorizontalOdometerInches()
        );
    }

    /**
     * Gets the current calculated 2D pose of the robot.
     * This is useful for 2D path following and field navigation.
     * @return A Pose2d object representing the robot's x, y, and heading.
     */
    public Pose2D getPose2D() {
        // Update the odometry to ensure the pose is current
        updateOdometry();

        // Return the 2D pose using ftclib geometry classes
        return new Pose2D(
                DistanceUnit.INCH, odometry.x, odometry.y,
                AngleUnit.DEGREES, headingProvider.getHeading()
        );
    }

    /**
     * Gets the current calculated pose of the robot after an update.
     * @return A Pose3D object representing the robot's current state.
     */
    public Pose3D getPose() {
        // First, update the odometry with the latest sensor readings
        updateOdometry();

        // Return the 3d pose
        Position position = new Position(DistanceUnit.INCH,
                odometry.x, odometry.y, 0, 0);
        YawPitchRollAngles orientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                headingProvider.getHeading(), 0, 0, 0);
        return new Pose3D(position, orientation);
    }
    
    /**
     * Calculates the power needed to turn to a target heading.
     * @param targetHeading The desired heading in degrees.
     * @param currentHeading The current heading in degrees.
     * @return The turn power, from -1.0 to 1.0.
     */
    public double calculateTurnPower(double targetHeading, double currentHeading) {
        double headingError = targetHeading - currentHeading;

        // Normalize the error to be within +/- 180 degrees for the shortest turn.
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Calculate the turning power using a proportional gain.
        // The Range.clip function limits the power to the range [-1, 1].
        double turnPower = Range.clip(headingError * TURN_GAIN, -1, 1);

        // Apply a minimum power to overcome static friction and prevent stalling.
        // Math.copySign ensures the direction of the minimum power is correct.
        return Math.copySign(Math.max(MIN_TURN_SPEED, Math.abs(turnPower)), turnPower);
    }


    /**
     * Turns the robot in place to a target heading.
     * @param targetHeading The desired heading in degrees.
     * @param maxPower The maximum power to use for the turn, from 0.0 to 1.0.
     */
    public void turnToHeading(double targetHeading, double maxPower) {
        // Calculate the initial heading error.
        double headingError = targetHeading - headingProvider.getHeading();

        // Loop until the robot is within the heading threshold and the opmode is active.
        while (opMode.opModeIsActive() && Math.abs(headingError) > HEADING_THRESHOLD) {
            double turnPower = calculateTurnPower(targetHeading, headingProvider.getHeading());

            // Limit the turn power to the maxPower
            turnPower = Range.clip(turnPower, -maxPower, maxPower);

            // Send power to the robot to make it turn. No forward or strafe movement.
            moveRobot(0, 0, turnPower);

            // Update odometry to keep track of the robot's position.
            updateOdometry();

            // Recalculate the heading error for the next loop iteration.
            headingError = targetHeading - headingProvider.getHeading();
        }

        // Stop all motion once the robot has reached the target heading.
        stopMotors();
    }

    public boolean driveTo(double targetX, double targetY, double maxPower) {
        // Get the current pose to calculate the initial error
        Pose2D currentPose = getPose2D();
        double errorX = targetX - currentPose.getX(DistanceUnit.INCH);
        double errorY = targetY - currentPose.getY(DistanceUnit.INCH);

        ElapsedTime timer = new ElapsedTime();
        double prevErr = Double.MAX_VALUE;
        double newErr = Math.hypot(errorX, errorY);

        // Loop until the robot is within the move threshold for both axes and the opmode is active
        while (opMode.opModeIsActive() && (newErr > MOVE_THRESHOLD_INCH)) {
            // Update the robot's current position and heading
            currentPose = getPose2D();
            double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);

            // Recalculate the field-centric error
            errorX = targetX - currentPose.getX(DistanceUnit.INCH);
            errorY = targetY - currentPose.getY(DistanceUnit.INCH);
            newErr = Math.hypot(errorX, errorY);
            Log.d(TAG+"driveTo", "err "+errorX+","+errorY+","+newErr);

            if (timer.seconds() > 0.1) {
                timer.reset();
                if (prevErr <= newErr) {
                    Log.d(TAG, "driveTo: OBSTRUCTION DETECTED. ABORT.");
                    stopMotors();
                    return false;
                }
                prevErr = newErr;
            }

            // --- Transform field-centric error into robot-centric power commands ---
            // This rotates the (errorX, errorY) vector from field coordinates to robot coordinates.
            double headingRad = currentPose.getHeading(AngleUnit.RADIANS);
            double sinH = Math.sin(headingRad);
            double cosH = Math.cos(headingRad);

            double forwardPower = (errorY * cosH) - (errorX * sinH);
            double strafePower  = (errorY * sinH) + (errorX * cosH);

            // --- Apply Proportional Gain ---
            // Scale down the power as the robot gets closer.
            // Adjust gain by the momentum.
            forwardPower *= MOVE_GAIN / maxPower;
            strafePower  *= MOVE_GAIN / maxPower;

            // --- Limit power to maxPower while preserving direction ---
            double largerPower = Math.max(Math.abs(forwardPower), Math.abs(strafePower));
            if (largerPower > maxPower) {
                double scale = maxPower / largerPower;
                forwardPower *= scale;
                strafePower *= scale;
            }

            // --- Apply Minimum Power to Overcome Friction ---
            // If the calculated power is very small but non-zero, boost it to the minimum
            // to ensure the robot actually moves. This avoids getting stuck near the target.
            if (newErr > MOVE_THRESHOLD_INCH) { // Only apply if we still need to move
                double minForwardPower = Math.min(maxPower, MIN_FORWARD_POWER);
                if (Math.abs(forwardPower) > 0.01 && Math.abs(forwardPower) < minForwardPower) {
                    forwardPower = Math.copySign(minForwardPower, forwardPower);
                }
                double minStrafePower = Math.min(maxPower, MIN_STRAFE_POWER);
                if (Math.abs(strafePower) > 0.01 && Math.abs(strafePower) < minStrafePower) {
                    strafePower = Math.copySign(minStrafePower, strafePower);
                }
            }

            // Send power to the robot. The 'turn' component is 0 to maintain heading.
            moveRobot(forwardPower, strafePower, 0);

            // Optional: Add telemetry for debugging
            opMode.telemetry.addData("Target", "X: %.2f, Y: %.2f", targetX, targetY);
            opMode.telemetry.addData("Pose", "X: %.2f, Y: %.2f, H: %.1f",
                    currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentHeading);
            opMode.telemetry.addData("Error", "X: %.2f, Y: %.2f", errorX, errorY);
            opMode.telemetry.addData("Power", "Fwd: %.2f, Str: %.2f", forwardPower, strafePower);
            opMode.telemetry.update();
        }

        // Stop all motion once the robot has reached the target location.
        stopMotors();
        return true;
    }

    /**
     * Moves the robot by a relative distance (dx, dy) from its current position.
     * @param dx The distance to move along the field's X-axis (in inches).     * @param dy The distance to move along the field's Y-axis (in inches).
     * @param power The maximum power to use for movement.
     * @return True if the movement completed successfully, false if it was obstructed.
     */
    public boolean driveRelative(double dx, double dy, double power) {
        // 1. Get the current pose to determine the starting point.
        Pose2D currentPose = getPose2D();
        double startX = currentPose.getX(DistanceUnit.INCH);
        double startY = currentPose.getY(DistanceUnit.INCH);

        // 2. Calculate the absolute target coordinates.
        double targetX = startX + dx;
        double targetY = startY + dy;

        // 3. Use the existing driveTo method to move to the calculated target.
        return driveTo(targetX, targetY, power);
    }


    /**
     * Sends power to the drivetrain motors. The inputs are robot-centric.
     * @param forward Power for forward/backward movement (+Y forward).
     * @param strafe Power for left/right movement (+X right).
     * @param turn Power for rotation (+ counter-clockwise).
     */
    public void moveRobot(double forward, double strafe, double turn) {
        // Calculate wheel powers using standard omni-drive kinematics.
        double leftFrontPower  = forward + strafe - turn;
        double rightFrontPower = forward - strafe + turn;
        double leftBackPower   = forward - strafe - turn;
        double rightBackPower  = forward + strafe + turn;

        // Find the maximum absolute power to use for normalization.
        // This ensures that no wheel power exceeds 1.0 while maintaining the intended drive ratios.
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        // Send normalized powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower / max);
        rightFrontDrive.setPower(rightFrontPower / max);
        leftBackDrive.setPower(leftBackPower / max);
        rightBackDrive.setPower(rightBackPower / max);
    }

    /**
     * Calculates and applies motor powers for human-centric driving.
     *
     * @param forward The forward/backward input from the joystick (-1 to 1).
     * @param strafe The left/right strafe input from the joystick (-1 to 1).
     * @param turn The rotational input from the joystick (-1 to 1).
     * @param robotHeadingDeg The robot's current absolute heading from the IMU, in degrees.
     * @param humanHeadingDeg The direction the human driver is facing, in degrees.
     */
    public void moveHumanCentric(double forward, double strafe, double turn, double robotHeadingDeg, double humanHeadingDeg) {
        // Get the robot's heading relative to the human's heading.
        double effectiveHeadingDeg = robotHeadingDeg - humanHeadingDeg;
        double headingRad = Math.toRadians(effectiveHeadingDeg);

        // Pre-calculate the sine and cosine of the effective heading.
        double sinH = Math.sin(headingRad);
        double cosH = Math.cos(headingRad);

        // Rotate the joystick input vector by the negative of the robot's effective heading.
        // This transforms the driver's human-centric commands into robot-centric commands
        // that the moveRobot() method can understand.
        double robotForward = (forward * cosH) - (strafe * sinH);
        double robotStrafe  = (forward * sinH) + (strafe * cosH);

        // Pass the calculated robot-centric power values and turn value to the drivetrain.
        moveRobot(robotForward, robotStrafe, turn);
    }

    /**
     * Stops all drivetrain motion.
     */
    public void stopMotors() {
        moveRobot(0, 0, 0);
    }

    /**
     * Gets the current position of the left odometer in inches.
     * @return Position in inches.
     */
    public double getLeftOdometerInches() {
        return leftOdometer.getCurrentPosition() * ODOMETER_INCH_PER_COUNT;
    }

    /**
     * Gets the current position of the right odometer in inches.
     * @return Position in inches.
     */
    public double getRightOdometerInches() {
        return rightOdometer.getCurrentPosition() * ODOMETER_INCH_PER_COUNT;
    }

    /**
     * Gets the current position of the horizontal odometer in inches.
     * @return Position in inches.
     */
    public double getHorizontalOdometerInches() {
        return horizontalOdometer.getCurrentPosition() * ODOMETER_INCH_PER_COUNT;
    }

    // --- Constants copied from VexIMUOmniDriveTrain ---
    static final double HEADING_THRESHOLD = 2.0 ;   // How close must the heading get to the target before moving to next step.
    static final double MIN_TURN_SPEED = 0.1;
    static final double TURN_GAIN = 1.0 / 15.0 ;    // Turn Control "Gain". Start reducing power at 15 degrees.

    static final double MOVE_THRESHOLD_INCH = 0.5;
    static final double MIN_FORWARD_POWER = 0.1;
    static final double MIN_STRAFE_POWER = 0.1;
    static final double MOVE_GAIN = 1.0 / 5.0;      // Move Control "Gain". Start reducing power at 5 inches assuming full power.

    // https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/
    static final double ODOMETER_DIAMETER_MM = 48;
    static final double ODOMETER_COUNT_PER_REVOLUTION = 2000;
    static final double ODOMETER_MM_PER_COUNT = (ODOMETER_DIAMETER_MM * Math.PI) / ODOMETER_COUNT_PER_REVOLUTION;
    static final double ODOMETER_INCH_PER_COUNT = ODOMETER_MM_PER_COUNT / 25.4;

    // Odometry Calibration: See VexOdometryCalibration.java
    static final double ODOMETER_TRACK_WIDTH_INCH = 8 + 3.0/8; // 8.375 in
    static final double ODOMETER_CENTER_WHEEL_OFFSET_INCH = -17.25/2 + (7+7.0/16); // -1.1875 in

    private static final String TAG = "VEX::";
}
