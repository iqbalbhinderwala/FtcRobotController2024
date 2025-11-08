package org.firstinspires.ftc.teamcode.Vex.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
@Disabled
public class HumanCentricTeleOp extends LinearOpMode {

    // Declare our drivetrain from the VexOdometryDriveTrain class
    private VexOdometryDriveTrain driveTrain;
    private VexVision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---

        // Define the driver's fixed orientation. 0 degrees means facing along the +Y axis (Red Alliace).
        double humanDirection = 0;


        // Initialize the drivetrain, which will handle all hardware mapping and setup.
        driveTrain = new VexOdometryDriveTrain(this);
        driveTrain.init();

        // Initialize vision
        vision = new VexVision(this);
        vision.init();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to start.");
        telemetry.update();
        
        AprilTagDetection detection = null;

        // Loop while waiting for start and look for AprilTags
        while (!isStarted() && !isStopRequested()) {
            detection = vision.getMostAccurateTarget();
            if (detection != null && detection.robotPose != null) {
                telemetry.addLine("AprilTag found! Pose will be initialized from this tag.");
                vision.addTelemetry();
            } else {
                telemetry.addLine("Searching for AprilTag... Press PLAY to start without one.");
            }
            telemetry.update();
            sleep(100);
        }

        // Don't do anything if Stop was requested
        if (isStopRequested()) {
             vision.stop();
             return;
        }

        // Re-check detection in case it was found right before starting.
        if (detection == null) {
            detection = vision.getMostAccurateTarget();
        }

        // Use vision to set pose if found, otherwise default to 0,0,0
        if (detection != null && detection.robotPose != null) {
            Pose3D robotPose = detection.robotPose;
            driveTrain.resetPose(robotPose.getPosition().x, robotPose.getPosition().y, robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.addLine("Pose initialized from AprilTag.");
        } else {
            driveTrain.resetPose(0, 0, 0);
            telemetry.addLine("No AprilTag found, starting at (0,0,0).");
        }
        telemetry.addData(">", "Press Play to start controlling the robot.");
        telemetry.addData(">", "Driver orientation is set to face the Negative X-Axis (+90 deg).");
        telemetry.update();

        // Wait for the driver to press the START button on the driver station.
        waitForStart();

        // --- TELEOP LOOP ---

        while (opModeIsActive()) {
            // Check for AprilTag detections to update pose
            AprilTagDetection newDetection = vision.getMostAccurateTarget();
            if (newDetection != null && newDetection.robotPose != null) {
                Pose3D robotPose = newDetection.robotPose;
                driveTrain.updatePosition(robotPose.getPosition().x, robotPose.getPosition().y);
                telemetry.addLine("!!! POSE UPDATED FROM VISION !!!");
            }

            // --- READ JOYSTICK INPUTS ---

            // Get raw joystick values for movement and turning.
            double forwardInput = -gamepad1.left_stick_y; // Positive is "forward"
            double strafeInput  =  gamepad1.left_stick_x;  // Positive is "right"
            double turnInput    = -gamepad1.right_stick_x; // Positive is counter-clockwise

            // --- CALL DRIVETRAIN METHOD ---

            // Get the robot's current heading from the IMU.
            double robotHeading = driveTrain.getHeading();

            // Call the centralized driving function within the drivetrain class,
            // passing all necessary inputs. The drivetrain now handles all calculations.
            driveTrain.moveHumanCentric(forwardInput, strafeInput, turnInput, robotHeading, humanDirection);

            // --- TELEMETRY ---

            updateTelemetry(forwardInput, strafeInput, turnInput);
        }

        // --- CLEANUP ---

        // Ensure the robot stops moving when the OpMode is stopped.
        driveTrain.stopMotors();
        vision.stop();
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
        
        vision.addTelemetry();
        
        telemetry.update();
    }
}
