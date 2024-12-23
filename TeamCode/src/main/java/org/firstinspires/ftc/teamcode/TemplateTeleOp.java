package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Template Tele Op", group="Template")
@Disabled
public class TemplateTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    TemplateExternalClass hardwareComponent = new TemplateExternalClass(this);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to start.");
        telemetry.update();

        // Wait for driver to press PLAY before robot allowed to move.
        waitForStart();
        runtime.reset();

        // Initialize/Start components.
        hardwareComponent.init();
        hardwareComponent.start();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
