package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="[Sam] *** MAIN AutoPark *** ", group="Sam")
//@Disabled
public class SamAutoPark extends LinearOpMode
{
    private SamEncoderDriveTrain nav = new SamEncoderDriveTrain(this, false);

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

        // STRAFE RIGHT 2 inches
        nav.driveByDistance(2, true, 0.4);

        // DRIVE BACK 26 inches
        nav.driveByDistance(-26, false, 0.4);

        while (opModeIsActive())
        {
            // Telemetry
            //telemetry.update();
            idle();
        }
        sleep(1000);  // Pause to display last telemetry message.
    }
}
