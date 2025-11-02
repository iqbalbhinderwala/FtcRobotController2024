package org.firstinspires.ftc.teamcode.Vex.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;

/**
 * This TeleOp program provides field-centric control for a robot using the VexOdometryDriveTrain class.
 *
 * Field-centric control means the robot's movement is relative to the driver and the field,
 * not the robot's front. "Forward" on the joystick will always move the robot away from the driver,
 * regardless of which way the robot is pointing.
 *
 * CONTROLS:
 * - Left Stick:  Controls strafing (left/right) and forward/backward movement.
 * - Right Stick: Controls the robot's rotation (turning).
 */
@TeleOp(name = "TeleOp - Field Centric", group = "Vex")
public class FieldCentricTeleOp extends LinearOpMode {

    // Declare our drivetrain from the VexOdometryDriveTrain class
    private VexOdometryDriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---

        // Initialize the drivetrain, which will handle all hardware mapping and setup.
        driveTrain = new VexOdometryDriveTrain(this);
        driveTrain.init();

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start controlling the robot.");
        telemetry.update();

        // Wait for the driver to press the START button on the driver station.
        waitForStart();

        // --- TELEOP LOOP ---

        while (opModeIsActive()) {
            // --- READ JOYSTICK INPUTS ---

            // Get raw joystick values. The y-axis is traditionally inverted on gamepads.
            double forwardInput = -gamepad1.left_stick_y; // Positive is forward
            double strafeInput  =  gamepad1.left_stick_x;  // Positive is right
            double turnInput    = -gamepad1.right_stick_x; // Positive is counter-clockwise

            // --- FIELD-CENTRIC CALCULATION ---

            // Get the robot's current heading in radians from the drivetrain's IMU provider.
            double headingRad = Math.toRadians(driveTrain.getHeading());
            double sinH = Math.sin(headingRad);
            double cosH = Math.cos(headingRad);

            // Rotate the joystick input vector (forward, strafe) by the negative of the robot's heading.
            // This transforms the driver's field-centric commands into robot-centric commands
            // that the moveRobot() method can understand.
            double robotForward = (forwardInput * cosH) - (strafeInput * sinH);
            double robotStrafe  = (forwardInput * sinH) + (strafeInput * cosH);

            // --- SEND POWER TO MOTORS ---

            // Pass the calculated robot-centric power values to the drivetrain.
            // The moveRobot method in your class will handle the rest (kinematics, normalization).
            driveTrain.moveRobot(robotForward, robotStrafe, turnInput);

            // --- TELEMETRY ---

            // Update odometry to get the latest position for display.
            driveTrain.update();

            // Display useful information on the driver station screen for debugging.
            telemetry.addData("--- Controls ---", "");
            telemetry.addData("Left Stick Y (Fwd)", "%.2f", forwardInput);
            telemetry.addData("Left Stick X (Str)", "%.2f", strafeInput);
            telemetry.addData("Right Stick X (Trn)", "%.2f", turnInput);
            telemetry.addData("--- Robot ---", "");
            telemetry.addData("Heading (Deg)", "%.1f", driveTrain.getHeading());
            telemetry.addData("Pose", driveTrain.getPose().toString());
            telemetry.update();
        }

        // --- CLEANUP ---
        // Ensure the robot stops moving when the OpMode is stopped.
        driveTrain.stopMotors();
    }
}

