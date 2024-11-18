package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Template Auto Op", group="Template")
@Disabled
public class TemplateAutoOp extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    TemplateExternalClass hardwareComponent = new TemplateExternalClass(this);

    @Override
    public void runOpMode()
    {
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to start.");
        telemetry.update();

        // Wait for driver to press PLAY before robot allowed to move.
        waitForStart();
        runtime.reset();

        // Initialize/Start components.
        hardwareComponent.init();
        hardwareComponent.start();

        // Started
        while (opModeIsActive())
        {
            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
