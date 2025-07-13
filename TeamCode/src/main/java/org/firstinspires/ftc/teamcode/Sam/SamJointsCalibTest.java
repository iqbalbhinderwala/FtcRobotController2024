package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="[Sam] Calibrate Joints", group="SamTest")
@Disabled
public class SamJointsCalibTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    SamJoints joints = new SamJoints(this);

    @Override
    public void runOpMode() {
        // Initialize components.
        joints.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Start components.
        joints.start();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double basePower   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative stick_y value
            double armPower    = -gamepad1.right_stick_y; // Note: pushing stick forward gives negative stick_y value
            double wristPower  = (-gamepad1.left_trigger + gamepad1.right_trigger);
            joints.actuate(basePower, armPower, wristPower);

            if (!joints.isFullyCalibrated())
            {
                joints.tryResetEncoders();
            }

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            joints.addTelemetry();
            if (joints.isFullyCalibrated()) {
                telemetry.addData("#", "*** FULLY CALIBRATED ***");
            }
            telemetry.update();
        }
    }
}
