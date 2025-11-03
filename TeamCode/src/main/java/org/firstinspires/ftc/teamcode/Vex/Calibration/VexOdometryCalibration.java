package org.firstinspires.ftc.teamcode.Vex.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;

/**
 * A TeleOp program for calibrating the odometry system.
 * This routine helps determine the robot's trackWidth and centerWheelOffset.
 *
 * Instructions:
 * 1. Place the robot on a flat, open surface.
 * 2. Press START on the Driver Station.
 * 3. Press the 'A' button to begin the automated calibration turn.
 * 4. The robot will rotate approximately 360 degrees and then stop.
 * 5. The calculated 'trackWidth' and 'centerWheelOffset' values will be displayed on the telemetry.
 * 6. Record these values and update them in your VexOdometryDriveTrain class.
 * 7. You can press 'A' to run the test again.
 */
@TeleOp(name="[Vex] Odometry Calibration", group="VexTest")
public class VexOdometryCalibration extends LinearOpMode {

    // Instantiate the drive train, which also handles odometry
    VexOdometryDriveTrain robot = new VexOdometryDriveTrain(this);
    private double cumulativeDegrees = 0;

    // Constants for the calibration routine
    final double CALIBRATION_TURN_SPEED = 0.7; // Power for turning during calibration
    final double TARGET_ANGLE_DEGREES_QUICK = 360.0; // The angle to turn (quick)
    final double TARGET_ANGLE_DEGREES_EXTRA = 10 * 360.0; // The angle to turn (slow)
    final double BUTTON_DELAY_SEC = 0.5; // Prevent accidental double-presses

    ElapsedTime buttonTimer = new ElapsedTime();
    private String calibrationStatus = "Ready";

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot's hardware
        robot.init();
        robot.resetPose(0, 0, 0);

        telemetry.addLine(">>> Odometry Calibration <<<");
        telemetry.addLine("Place robot on a flat surface.");
        telemetry.addLine("Press 'A' to start the 360-degree rotation test.");
        telemetry.addLine("Press 'X' to start the 10x360-degree rotation test.");
        telemetry.update();

        waitForStart();

        // Reset the button timer after start is pressed
        buttonTimer.reset();

        while (opModeIsActive()) {
            // Check if the 'A' button is pressed to start calibration
            if (gamepad1.a && buttonTimer.seconds() > BUTTON_DELAY_SEC) {
                buttonTimer.reset();
                runCalibrationRoutine(TARGET_ANGLE_DEGREES_QUICK);
            }
            // Check if the 'X' button is pressed to start calibration
            if (gamepad1.x && buttonTimer.seconds() > BUTTON_DELAY_SEC) {
                buttonTimer.reset();
                runCalibrationRoutine(TARGET_ANGLE_DEGREES_EXTRA);
            }
            // Display live telemetry data
            displayTelemetry();
        }

        // Ensure motors are off when the OpMode is stopped
        robot.stopMotors();
    }

    /**
     * Executes the automated calibration sequence.
     */
    private void runCalibrationRoutine(double targetAnglesDeg) {
        // 1. Prepare for calibration
        calibrationStatus = "Calibrating...";
        displayTelemetry();

        // Reset all relevant sensors: IMU yaw and encoder counts
        robot.init();
        double previousHeading = robot.getHeading();
        cumulativeDegrees = 0;

        // 2. Start rotating the robot
        // A positive turn value results in a counter-clockwise rotation.
        robot.moveRobot(0, 0, CALIBRATION_TURN_SPEED);

        boolean complete_loop_and_exit = false;

        // 3. Loop until the target angle is reached
        while (opModeIsActive() && !complete_loop_and_exit) {
            if (cumulativeDegrees >= targetAnglesDeg) {
                calibrationStatus = "Target achieved! Stopping Motors";
                // 4. Stop the robot's movement
                robot.stopMotors();
                // Small delay to ensure the robot has fully stopped before reading final values
                sleep(250);
                // Update the final cumulative degrees before breaking out of the loop
                complete_loop_and_exit = true;
            }

            double currentHeading = robot.getHeading();
            double deltaDegrees = currentHeading - previousHeading;
            while (deltaDegrees > 180)  deltaDegrees -= 360;
            while (deltaDegrees <= -180) deltaDegrees += 360;
            cumulativeDegrees += deltaDegrees;
            previousHeading = currentHeading;

            // The robot continues to turn. Displaying telemetry helps monitor progress.
            displayTelemetry();
        }

        // 5. Retrieve final sensor readings after the turn
        double finalAngleRad = Math.toRadians(cumulativeDegrees);
        double deltaLeft = robot.getLeftOdometerInches();
        double deltaRight = robot.getRightOdometerInches();
        double deltaHorizontal = robot.getHorizontalOdometerInches();

        // 6. Calculate the calibration values
        // Avoid division by zero if the robot didn't turn
        if (Math.abs(finalAngleRad) > Math.toRadians(1)) { // Check if it turned at least 1 degree
            // For a counter-clockwise turn, left encoder will be negative, right will be positive.
            // The difference (deltaRight - deltaLeft) gives the total arc distance.
            double calculatedTrackWidth = (deltaRight - deltaLeft) / finalAngleRad;

            // The horizontal movement is directly proportional to the offset and the angle turned.
            // To match the TwistOdometry convention (where offset is the physical Y-coordinate),
            //      pure turn: 0 == deltaHorizontal + (centerWheelOffset * angleRad)
            double calculatedCenterWheelOffset = -deltaHorizontal / finalAngleRad;

            // Update status and show results
            calibrationStatus = "Complete! Record these values:";
            telemetry.addData(">>> Calculated TrackWidth", "%.4f inches", calculatedTrackWidth);
            telemetry.addData(">>> Calculated CenterWheelOffset", "%.4f inches", calculatedCenterWheelOffset);
        } else {
            calibrationStatus = "Error: Robot did not turn enough.";
        }

        telemetry.addLine("Press 'B' to continue.");
        telemetry.update();
        while(opModeIsActive() && !gamepad1.b) {
            sleep(250);
        }
    }

    /**
     * Displays relevant telemetry data on the Driver Station.
     */
    private void displayTelemetry() {
        telemetry.addLine("Press 'A' or 'X' to start calibration.");
        telemetry.addData("Status", calibrationStatus);
        telemetry.addLine("--- Live Data ---");
        telemetry.addData("Heading (Deg)", "%.2f", robot.getHeading());
        telemetry.addData("Left Pod (in)", "%.2f", robot.getLeftOdometerInches());
        telemetry.addData("Right Pod (in)", "%.2f", robot.getRightOdometerInches());
        telemetry.addData("Horizontal Pod (in)", "%.2f", robot.getHorizontalOdometerInches());
        telemetry.addData("Cumulative turn angles (deg)", "%.2f", cumulativeDegrees);
        telemetry.update();
    }
}
