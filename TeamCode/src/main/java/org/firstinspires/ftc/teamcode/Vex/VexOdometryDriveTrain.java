package org.firstinspires.ftc.teamcode.Vex;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// FROM VexIMUOmniDriveTrain.java

public class VexOdometryDriveTrain {

    // Parent OpMode
    private LinearOpMode opMode;

    // Core Components
    private IMUHeadingProvider headingProvider;
    private LinearOdometry odometry;

    // Hardware
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor leftOdometer, rightOdometer, horizontalOdometer;


    /**
     * Constructor for the drivetrain.
     * @param opMode The parent LinearOpMode.
     */
    public VexOdometryDriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;
        this.odometry = new LinearOdometry();
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
        leftFrontDrive = hardwareMap.get(DcMotor.class, "wheel front left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "wheel front right");
        leftBackDrive = hardwareMap.get(DcMotor.class, "wheel back left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "wheel back right");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            // TurnToHeading() relies on encoders being disabled.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- Initialize Odometry Pods ---
        leftOdometer = hardwareMap.get(DcMotor.class, "odometer axial left");
        rightOdometer = hardwareMap.get(DcMotor.class, "odometer axial right");
        horizontalOdometer = hardwareMap.get(DcMotor.class, "odometer lateral");

        leftOdometer.setDirection(DcMotor.Direction.FORWARD); // Y +-ve forward
        rightOdometer.setDirection(DcMotor.Direction.REVERSE); // Y +-ve forward
        horizontalOdometer .setDirection(DcMotor.Direction.FORWARD); // X +-ve right

        // Reset all odometer encoders
        for (DcMotor motor : new DcMotor[]{leftOdometer, rightOdometer, horizontalOdometer}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); // Reset the IMU heading.

        headingProvider = new IMUHeadingProvider(imu);
    }

    /**
     * Sets the robot's starting position and heading on the field.
     * @param startX Initial field X-coordinate in inches.
     * @param startY Initial field Y-coordinate in inches.
     * @param startHeading Initial field heading in degrees.
     */
    public void setPose(double startX, double startY, double startHeading) {
        odometry.initPose(startX, startY,
                getLeftOdometerInches(), getRightOdometerInches(), getHorizontalOdometerInches());
        headingProvider.resetYaw(startHeading);
    }

    /**
     * Gets the current calculated pose of the robot after an update.
     * @return A Pose3D object representing the robot's current state.
     */
    public Pose3D getPose() {
        // First, update the odometry with the latest sensor readings
        odometry.update(
                headingProvider.getHeading(),
                getLeftOdometerInches(),
                getRightOdometerInches(),
                getHorizontalOdometerInches()
        );

        // Return the 3d pose
        Position position = new Position(DistanceUnit.INCH,
                odometry.x, odometry.y, 0, 0);
        YawPitchRollAngles orientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                headingProvider.getHeading(), 0, 0, 0);
        return new Pose3D(position, orientation);
    }


    /**
     * Turns the robot in place to a target heading.
     * @param targetHeading The desired heading in degrees.
     */
    public void turnToHeading(double targetHeading) {
        // Calculate the initial heading error.
        double headingError = targetHeading - headingProvider.getHeading();

        // Loop until the robot is within the heading threshold and the opmode is active.
        while (opMode.opModeIsActive() && Math.abs(headingError) > HEADING_THRESHOLD) {
            // Normalize the error to be within +/- 180 degrees for the shortest turn.
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Calculate the turning power using a proportional gain.
            // The Range.clip function limits the power to the range [-1, 1].
            double turnPower = com.qualcomm.robotcore.util.Range.clip(headingError * TURN_GAIN, -1, 1);

            // Apply a minimum power to overcome static friction and prevent stalling.
            // Math.copySign ensures the direction of the minimum power is correct.
            turnPower = Math.copySign(Math.max(MIN_TURN_SPEED, Math.abs(turnPower)), turnPower);

            // Send power to the robot to make it turn. No forward or strafe movement.
            moveRobot(0, 0, turnPower);

            // Recalculate the heading error for the next loop iteration.
            headingError = targetHeading - headingProvider.getHeading();
        }

        // Stop all motion once the robot has reached the target heading.
        stopMotors();
    }

    public void driveTo(double targetX, double targetY) {
        // Get the current pose to calculate the initial error
        Pose3D currentPose = getPose();
        double errorX = targetX - currentPose.getPosition().x;
        double errorY = targetY - currentPose.getPosition().y;

        // Loop until the robot is within the move threshold for both axes and the opmode is active
        while (opMode.opModeIsActive() && (Math.abs(errorX) > MOVE_THRESHOLD_INCH || Math.abs(errorY) > MOVE_THRESHOLD_INCH)) {
            // Update the robot's current position and heading
            currentPose = getPose();
            double currentHeading = currentPose.getOrientation().getYaw(AngleUnit.DEGREES);

            // Recalculate the field-centric error
            errorX = targetX - currentPose.getPosition().x;
            errorY = targetY - currentPose.getPosition().y;

            // --- Transform field-centric error into robot-centric power commands ---
            // This rotates the (errorX, errorY) vector from field coordinates to robot coordinates.
            double headingRad = Math.toRadians(currentHeading);
            double sinH = Math.sin(headingRad);
            double cosH = Math.cos(headingRad);

            double forwardPower = (errorY * cosH) - (errorX * sinH);
            double strafePower  = (errorY * sinH) + (errorX * cosH);

            // --- Apply Proportional Gain ---
            // Scale down the power as the robot gets closer.
            forwardPower *= MOVE_GAIN;
            strafePower  *= MOVE_GAIN;

            // --- Normalize the Power Vector ---
            // This is the crucial change. Instead of clipping each power component independently,
            // we scale them down together. This preserves the desired angle of travel
            // even when the requested power exceeds 1.0, preventing the robot from being
            // locked into a 45-degree path.
            double max = Math.max(1.0, Math.abs(forwardPower) + Math.abs(strafePower));
            forwardPower /= max;
            strafePower  /= max;

            // --- Apply Minimum Power to Overcome Friction ---
            // If the calculated power is very small but non-zero, boost it to the minimum
            // to ensure the robot actually moves. This avoids getting stuck near the target.
            if (Math.hypot(errorX, errorY) > MOVE_THRESHOLD_INCH) { // Only apply if we still need to move
                if (Math.abs(forwardPower) > 0.01 && Math.abs(forwardPower) < MIN_MOVE_POWER) {
                    forwardPower = Math.copySign(MIN_MOVE_POWER, forwardPower);
                }
                if (Math.abs(strafePower) > 0.01 && Math.abs(strafePower) < MIN_MOVE_POWER) {
                    strafePower = Math.copySign(MIN_MOVE_POWER, strafePower);
                }
            }

            // Send power to the robot. The 'turn' component is 0 to maintain heading.
            moveRobot(forwardPower, strafePower, 0);

            // Optional: Add telemetry for debugging
            opMode.telemetry.addData("Target", "X: %.2f, Y: %.2f", targetX, targetY);
            opMode.telemetry.addData("Pose", "X: %.2f, Y: %.2f, H: %.1f",
                    currentPose.getPosition().x, currentPose.getPosition().y, currentHeading);
            opMode.telemetry.addData("Error", "X: %.2f, Y: %.2f", errorX, errorY);
            opMode.telemetry.addData("Power", "Fwd: %.2f, Str: %.2f", forwardPower, strafePower);
            opMode.telemetry.update();
        }

        // Stop all motion once the robot has reached the target location.
        stopMotors();
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
     * Stops all drivetrain motion.
     */
    private void stopMotors() {
        moveRobot(0, 0, 0);
    }

    /**
     * Gets the current position of the left odometer in inches.
     * @return Position in inches.
     */
    private double getLeftOdometerInches() {
        return leftOdometer.getCurrentPosition() * ODOMETER_INCH_PER_COUNT;
    }

    /**
     * Gets the current position of the right odometer in inches.
     * @return Position in inches.
     */
    private double getRightOdometerInches() {
        return rightOdometer.getCurrentPosition() * ODOMETER_INCH_PER_COUNT;
    }

    /**
     * Gets the current position of the horizontal odometer in inches.
     * @return Position in inches.
     */
    private double getHorizontalOdometerInches() {
        return horizontalOdometer.getCurrentPosition() * ODOMETER_INCH_PER_COUNT;
    }

    // --- Constants copied from VexIMUOmniDriveTrain ---
    static final double HEADING_THRESHOLD = 2.0 ;   // How close must the heading get to the target before moving to next step.
    static final double MIN_TURN_SPEED = 0.1;
    static final double TURN_GAIN = 1.0 / 15.0 ;    // Turn Control "Gain". Start reducing power at 15 degrees.

    static final double MOVE_THRESHOLD_INCH = 0.5;
    static final double MIN_MOVE_POWER = 0.1;
    static final double MOVE_GAIN = 1.0 / 3.0;      // Move Control "Gain". Start reducing power at 3 inches

    // https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/
    static final double ODOMETER_DIAMETER_MM = 48;
    static final double ODOMETER_COUNT_PER_REVOLUTION = 2000;
    static final double ODOMETER_MM_PER_COUNT = (ODOMETER_DIAMETER_MM * Math.PI) / ODOMETER_COUNT_PER_REVOLUTION;
    static final double ODOMETER_INCH_PER_COUNT = ODOMETER_MM_PER_COUNT / 25.4;
}
