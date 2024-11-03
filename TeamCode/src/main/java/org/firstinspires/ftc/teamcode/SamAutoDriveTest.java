package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Simple autonomous program to test driving in a straight line.

@Autonomous(name="[Sam] TEST AUTO --> BACKUP HW", group="SamTest")
//@Disabled
public class SamAutoDriveTest extends LinearOpMode
{
    private SamEncoderDriveTrain nav = new SamEncoderDriveTrain(this, true);
    double MAX_DRIVE_SPEED = 0.4;
    ElapsedTime lastClick = new ElapsedTime();
    double BUTTON_DELAY_SEC = 0.25;

    double FORWARD_DIST_INCH = 26;
    double SIDE_DIST_INCH = 2;

    @Override
    public void runOpMode()
    {
        nav.init();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to start.");
        telemetry.update();

        // Wait for driver to press PLAY
        waitForStart();

        // Started
        while (opModeIsActive())
        {
            if (lastClick.seconds() > BUTTON_DELAY_SEC) {
                // forward
                if (gamepad1.dpad_up) {
                    lastClick.reset();
                    nav.driveByDistance(FORWARD_DIST_INCH, false, MAX_DRIVE_SPEED);
                }
                // right
                if (gamepad1.dpad_right) {
                    lastClick.reset();
                    nav.driveByDistance(SIDE_DIST_INCH, true, MAX_DRIVE_SPEED);
                }
                // back
                if (gamepad1.dpad_down) {
                    lastClick.reset();
                    nav.driveByDistance(-FORWARD_DIST_INCH, false, MAX_DRIVE_SPEED);
                }
                // left
                if (gamepad1.dpad_left) {
                    lastClick.reset();
                    nav.driveByDistance(-SIDE_DIST_INCH, true, MAX_DRIVE_SPEED);
                }
            }

            // Telemetry
            telemetry.addData("Drive Speed", "%.2f", MAX_DRIVE_SPEED);
            telemetry.update();
        }
    }
}
