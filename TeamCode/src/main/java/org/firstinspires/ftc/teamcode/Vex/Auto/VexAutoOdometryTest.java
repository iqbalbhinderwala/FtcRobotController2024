package org.firstinspires.ftc.teamcode.Vex.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;

// Simple program to test driving to coordinates and turning to a heading.

@Autonomous(name="[Vex] Odometry Test", group="VexTest")
public class VexAutoOdometryTest extends LinearOpMode
{
    VexOdometryDriveTrain robot = new VexOdometryDriveTrain(this);
    double MAX_DRIVE_POWER = 0.5;
    double MAX_TURN_SPEED = 0.5;
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
        robot.resetPose(0, 0, 0);

        double maxDrivePower = MAX_DRIVE_POWER;

        // Started
        while (opModeIsActive())
        {
            // Select max speed (don't drive full power)
            if(gamepad1.dpad_up && lastClick.seconds() >= BUTTON_DELAY_SEC) {
                lastClick.reset();
                maxDrivePower += 0.1;
                maxDrivePower =  Range.clip(maxDrivePower, 0, 1);
            } else if (gamepad1.dpad_down && lastClick.seconds() >= BUTTON_DELAY_SEC) {
                lastClick.reset();
                maxDrivePower -= 0.1;
                maxDrivePower =  Range.clip(maxDrivePower, 0, 1);
            }

            // Drive using manual POV Joystick mode.  Slow things down to make the robot more controllable.
            double drive  = -gamepad1.left_stick_y  * maxDrivePower; // Note: pushing stick forward gives negative value so negate for +ve forward
            double strafe =  gamepad1.left_stick_x  * maxDrivePower; // +ve right
            double turn   = -gamepad1.right_stick_x * MAX_TURN_SPEED; // +ve counter-clockwise

            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            robot.moveRobot(drive, strafe, turn);

            // Rotate robot 90 degrees using DPAD left/right buttons.
            // turn 90 degrees right.
            if(gamepad1.dpad_right) {
                targetHeading -= 90;
                robot.turnToHeading(targetHeading, MAX_TURN_SPEED);
            }
            // turn 90 degrees left.
            if(gamepad1.dpad_left) {
                targetHeading += 90;
                robot.turnToHeading(targetHeading, MAX_TURN_SPEED);
            }
            if(gamepad1.a) {
                robot.driveTo(0, 0, maxDrivePower);
            }
            if(gamepad1.y) {
                robot.driveTo(0, 10, maxDrivePower);
            }
            if(gamepad1.x) {
                robot.driveTo(-5, 5, maxDrivePower);
            }
            if(gamepad1.b) {
                robot.driveTo(5, 5, maxDrivePower);
            }

            // Telemetry
            Pose3D pose = robot.getPose();
            telemetry.addLine("=== Controls ===");
            telemetry.addLine("A: Drive to ( 0,  0)");
            telemetry.addLine("Y: Drive to ( 0, 10)");
            telemetry.addLine("X: Drive to (-5,  5)");
            telemetry.addLine("B: Drive to ( 5,  5)");
            telemetry.addLine("D-Pad Left/Right: Turn 90°");
            telemetry.addLine("D-Pad Up/Down: Adjust drive speed");
            telemetry.addData("Drive Speed", "%.2f", maxDrivePower);
            telemetry.addData("Turn Speed", "%.2f", MAX_TURN_SPEED);
            telemetry.addData("X", "%.1f", pose.getPosition().x);
            telemetry.addData("Y", "%.1f", pose.getPosition().y);
            telemetry.addData("Heading", "%.1f", pose.getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        // Turn the motors off.
        robot.stopMotors();
    }
}
