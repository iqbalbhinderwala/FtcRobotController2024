package org.firstinspires.ftc.teamcode.Vex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sam.SamIMUOmniDriveTrain;

// Simple autonomous program to test driving in a straight line or rotate 90 degrees.

@Autonomous(name="[Vex] Auto Drive Sample", group="VexTest")
//@Disabled
public class VexAutoDriveSample extends LinearOpMode
{
    VexIMUOmniDriveTrain robot = new VexIMUOmniDriveTrain(this);
    double MAX_DRIVE_POWER = 0.7;
    double DISTANCE_INCH = 10;
    double MAX_TURN_SPEED = 1;
    double targetHeading = 0; // angle in degrees

    ElapsedTime lastClick = new ElapsedTime();
    double BUTTON_DELAY_SEC = 0.25;
    ElapsedTime driveTimer = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to start.");
        telemetry.update();

        // Wait for driver to press PLAY before robot allowed to move.
        waitForStart();

        robot.init();

        // Started
        while (opModeIsActive())
        {
            // Drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            double drive  = -gamepad1.left_stick_y  * MAX_DRIVE_POWER;
            double strafe = -gamepad1.left_stick_x  * MAX_DRIVE_POWER;
            double turn   = -gamepad1.right_stick_x * MAX_TURN_SPEED;
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            robot.moveRobot(drive, strafe, turn);

            // Rotate robot 90 degrees using DPAD left/right buttons.
            // turn 90 degrees right.
            if(gamepad1.dpad_right) {
                targetHeading -= 90;
                robot.turnToHeading(MAX_TURN_SPEED, targetHeading);
            }
            // turn 90 degrees left.
            if(gamepad1.dpad_left) {
                targetHeading += 90;
                robot.turnToHeading(MAX_TURN_SPEED, targetHeading);
            }
            // move forward
            if(gamepad1.y) {
                robot.driveDistance(DISTANCE_INCH, 0, MAX_DRIVE_POWER);
            }
            // move back
            if(gamepad1.a) {
                robot.driveDistance(-DISTANCE_INCH, 0, MAX_DRIVE_POWER);
            }
            // strafe left
            if(gamepad1.x) {
                robot.driveDistance(0, DISTANCE_INCH, MAX_DRIVE_POWER);
            }
            // strafe right
            if(gamepad1.b) {
                robot.driveDistance(0, -DISTANCE_INCH, MAX_DRIVE_POWER);
            }
            // diagonal forward-left
            if(gamepad1.left_bumper) {
                robot.driveDistance(DISTANCE_INCH, DISTANCE_INCH, MAX_DRIVE_POWER);
            }
            // diagonal forward-right
            if(gamepad1.right_bumper) {
                robot.driveDistance(DISTANCE_INCH, -DISTANCE_INCH, MAX_DRIVE_POWER);
            }
            // diagonal backward-left
            if(gamepad1.left_trigger > 0.5) {
                robot.driveDistance(-DISTANCE_INCH, DISTANCE_INCH, MAX_DRIVE_POWER);
            }
            // diagonal backward-right
            if(gamepad1.right_trigger > 0.5) {
                robot.driveDistance(-DISTANCE_INCH, -DISTANCE_INCH, MAX_DRIVE_POWER);
            }

            // Telemetry
            telemetry.addLine("=== Controls ===");
            telemetry.addLine("Y: Forward | A: Back");
            telemetry.addLine("X: Left | B: Right");
            telemetry.addLine("D-Pad Left/Right: Turn 90°");
            telemetry.addLine("Bumpers: Diagonal Forward (L/R)");
            telemetry.addLine("Triggers: Diagonal Back (L/R)");
            telemetry.addData("Distance", "%.1f inches", DISTANCE_INCH);
            telemetry.addData("Drive Speed", "%.2f", MAX_DRIVE_POWER);
            telemetry.addData("Turn Speed", "%.2f", MAX_TURN_SPEED);
            telemetry.addData("Heading error", "%.1f", targetHeading-robot.getHeading());
            telemetry.update();
        }

        // Turn the motors off.
        robot.stopMotors();
    }
}
