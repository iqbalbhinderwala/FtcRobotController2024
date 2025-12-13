package org.firstinspires.ftc.teamcode.Vex.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vex.Hardware.VexLeds;

@TeleOp(name="[Vex] LEDs Hardware Test", group="VexTest")
//@Disabled
public class VexLedsHWTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private VexLeds vexLeds;

    @Override
    public void runOpMode() {
        // Initialize the hardware component
        vexLeds = new VexLeds(this);
        vexLeds.init();

        telemetry.addData("Status", "Ready to start.");
        telemetry.update();

        // Wait for driver to press PLAY before robot allowed to move.
        waitForStart();
        runtime.reset();

        // Start the component (if any start logic exists)
        vexLeds.start();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Map gamepad buttons to LEDs
            // A -> LED 0
            // B -> LED 1
            // X -> LED 2
            // Y -> LED 3
            boolean led0State = gamepad1.a;
            boolean led1State = gamepad1.b;
            boolean led2State = gamepad1.x;
            boolean led3State = gamepad1.y;

            // Apply state to LEDs if they were found in hardwareMap
            if (vexLeds.getLed(0) != null) vexLeds.getLed(0).enable(led0State);
            if (vexLeds.getLed(1) != null) vexLeds.getLed(1).enable(led1State);
            if (vexLeds.getLed(2) != null) vexLeds.getLed(2).enable(led2State);
            if (vexLeds.getLed(3) != null) vexLeds.getLed(3).enable(led3State);

            // Show the elapsed game time and LED status.
            telemetry.addData("LED 0 (A)", vexLeds.getLed(0) != null ? (led0State ? "ON" : "OFF") : "NULL");
            telemetry.addData("LED 1 (B)", vexLeds.getLed(1) != null ? (led1State ? "ON" : "OFF") : "NULL");
            telemetry.addData("LED 2 (X)", vexLeds.getLed(2) != null ? (led2State ? "ON" : "OFF") : "NULL");
            telemetry.addData("LED 3 (Y)", vexLeds.getLed(3) != null ? (led3State ? "ON" : "OFF") : "NULL");
            telemetry.update();
        }
    }
}
