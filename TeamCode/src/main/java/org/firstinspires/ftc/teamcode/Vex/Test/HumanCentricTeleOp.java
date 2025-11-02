package org.firstinspires.ftc.teamcode.Vex.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;

/**
 * This TeleOp program provides human-centric control for a robot by utilizing the
 * moveHumanCentric method within the VexOdometryDriveTrain class.
 *
 * It assumes the driver is in a fixed position (e.g., facing +Y == zero heading).
 *
 * CONTROLS:
 * - Left Stick:  Controls strafing and forward/backward movement from the driver's perspective.
 * - Right Stick: Controls the robot's rotation (turning).
 */
@TeleOp(name = "TeleOp - Human Centric", group = "Vex")
public class HumanCentricTeleOp extends LinearOpMode {

    // Declare our drivetrain from the VexOdometryDriveTrain class
    private VexOdometryDriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---

        // Initialize the drivetrain, which will handle all hardware mapping and setup.
        driveTrain = new VexOdometryDriveTrain(this);
        driveTrain.init();

        // --- SET INITIAL POSE ---
        // Set the robot's starting position and heading on the field to (0, 0, 0).
        double startX = 0.0; // inches
        double startY = 0.0; // inches
        double startHeading = 0.0; // degrees
        driveTrain.setPose(0, 0, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start controlling the robot.");
        telemetry.addData(">", "Driver orientation is set to face the Negative X-Axis (+90 deg).");
        telemetry.addData("Initial Pose", "X: %.1f, Y: %.1f, H: %.1f", startX, startY, startHeading);
        telemetry.update();

        // Wait for the driver to press the START button on the driver station.
        waitForStart();

        // --- TELEOP LOOP ---

        while (opModeIsActive()) {
            // --- READ JOYSTICK INPUTS ---

            // Get raw joystick values for movement and turning.
            double forwardInput = -gamepad1.left_stick_y; // Positive is "forward"
            double strafeInput  =  gamepad1.left_stick_x;  // Positive is "right"
            double turnInput    = -gamepad1.right_stick_x; // Positive is counter-clockwise

            // --- CALL DRIVETRAIN METHOD ---

            // Get the robot's current heading from the IMU.
            double robotHeading = driveTrain.getHeading();

            // Define the driver's fixed orientation. 0 degrees means facing along the +Y axis.
            double humanDirection = +90.0;

            // Call the centralized driving function within the drivetrain class,
            // passing all necessary inputs. The drivetrain now handles all calculations.
            driveTrain.moveHumanCentric(forwardInput, strafeInput, turnInput, robotHeading, humanDirection);

            // --- TELEMETRY ---
            updateTelemetry(forwardInput, strafeInput, turnInput);
        }

        // --- CLEANUP ---
        // Ensure the robot stops moving when the OpMode is stopped.
        driveTrain.stopMotors();
    }

    /**
     * Updates and displays telemetry data on the Driver Station.
     */
    private void updateTelemetry(double fwd, double str, double trn) {
        // Update odometry to get the latest position for display.
        driveTrain.update();

        telemetry.addData("--- Controls ---", "");
        telemetry.addData("Left Stick Y (Fwd)", "%.2f", fwd);
        telemetry.addData("Left Stick X (Str)", "%.2f", str);
        telemetry.addData("Right Stick X (Trn)", "%.2f", trn);
        telemetry.addData("--- Robot ---", "");
        telemetry.addData("Heading (Deg)", "%.1f", driveTrain.getHeading());
        telemetry.addData("Pose", driveTrain.getPose().toString());
        telemetry.update();
    }
}

