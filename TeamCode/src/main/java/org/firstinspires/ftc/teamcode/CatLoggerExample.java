package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CatLoggerExample", group = "Examples")
@Disabled
public class CatLoggerExample extends LinearOpMode {

    private static final String TAG = "CatLogger::"; // Define your tag

    @Override
    public void runOpMode() {
        waitForStart();

        // Log Robot started
        Log.d(TAG, "OP Started");

        String last_state = "";
        while (opModeIsActive()) {
            String state = GamepadSerializer.serialize(gamepad1);
            // Only log gamepad state changes
            if (!state.equals(last_state)) {
                Log.d(TAG+"gamepad1", state);
                last_state = state;
            }

            telemetry.addData("Status", "Logging to Logcat...");
            telemetry.addData("Tag", TAG);
            telemetry.update();
        }

        // Log Robot stopped
        Log.d(TAG, "OP Stopped");
    }
}
