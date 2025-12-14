package org.firstinspires.ftc.teamcode.Vex.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "[Vex] Turn Step Response Recorder", group = "VexTest")
@Disabled
public class VexTurnRecorder extends LinearOpMode {

    // Data class for high-frequency time sampling
    private static class TurnDataPoint {
        final double timeSeconds;
        final double currentHeading;
        final double targetHeading;
        final double error;
        final double turnSpeed; // Angular Velocity
        final double power;
        final double gain;

        TurnDataPoint(double timeSeconds, double currentHeading, double targetHeading,
                      double error, double turnSpeed, double power, double gain) {
            this.timeSeconds = timeSeconds;
            this.currentHeading = currentHeading;
            this.targetHeading = targetHeading;
            this.error = error;
            this.turnSpeed = turnSpeed;
            this.power = power;
            this.gain = gain;
        }

        @Override
        public String toString() {
            // CSV Format: Time, CurrentHeading, TargetHeading, Error, TurnSpeed, Power, Gain
            return String.format(Locale.US, "%.4f,%.2f,%.2f,%.2f,%.2f,%.3f,%.4f",
                    timeSeconds, currentHeading, targetHeading, error, turnSpeed, power, gain);
        }
    }

    private VexOdometryDriveTrain driveTrain;
    private final ElapsedTime buttonTimer = new ElapsedTime();

    // Configuration constants
    private static final double RECORDING_DURATION = 5.0; // Seconds to record
    private static final double TEST_TURN_ANGLE = 90.0; // Target delta degrees

    // Tuning Variable (This updates the static variable in your hardware class if public,
    // or acts as a local override for testing)
    public static double CURRENT_TEST_GAIN = 0.02;

    @Override
    public void runOpMode() {
        driveTrain = new VexOdometryDriveTrain(this);
        driveTrain.init();

        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Ready for Turn Response Calibration.");
        telemetry.addLine("Press (A) to start a 90 deg turn sequence.");
        telemetry.addLine("Use D-Pad Up/Down to adjust Gain.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Allow adjusting gain on the fly before the test
            if (gamepad1.dpad_up && buttonTimer.seconds() > 0.2) {
                CURRENT_TEST_GAIN += 0.001;
                buttonTimer.reset();
            } else if (gamepad1.dpad_down && buttonTimer.seconds() > 0.2) {
                CURRENT_TEST_GAIN -= 0.001;
                buttonTimer.reset();
            }

            if (gamepad1.a && buttonTimer.seconds() > 1.0) {
                // Temporarily update the Hardware class gain if it's accessible,
                // otherwise we use our local gain in the loop below.
//                VexOdometryDriveTrain.TURN_GAIN = CURRENT_TEST_GAIN;

                runTurnSequence(TEST_TURN_ANGLE);
                buttonTimer.reset();
            }

            telemetry.addLine("Ready.");
            telemetry.addData("Target Delta", "%.1f", TEST_TURN_ANGLE);
            telemetry.addData("Current Gain (P)", "%.4f", CURRENT_TEST_GAIN);
            telemetry.addLine("Press (A) to start sequence.");
            telemetry.update();
        }
    }

    private void runTurnSequence(double targetDelta) {
        String timestampBase = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());

        // 1. Reset Phase
        telemetry.addLine("Resetting Pose...");
        telemetry.update();

        // Reset pose to 0 heading for a clean step response test
        driveTrain.resetPose(0, 0, 0);
        sleep(500); // Wait for IMU to settle if needed

        double startHeading = driveTrain.getHeading();
        double targetHeading = startHeading + targetDelta;

        // 2. Recording & Execution Phase
        telemetry.addData("Status", "Turning...");
        telemetry.update();

        List<TurnDataPoint> stepData = recordTurnResponse(targetHeading);

        // 3. Write Data Phase
        String filename = String.format(Locale.US, "TurnStep_Deg%.0f_Gain%.4f_%s.csv",
                targetDelta, CURRENT_TEST_GAIN, timestampBase);

        List<String> csvLines = new ArrayList<>();
        for (TurnDataPoint point : stepData) {
            csvLines.add(point.toString());
        }

        writeAllDataToFile(filename, csvLines);

        // Stop motors after test
        driveTrain.moveRobot(0, 0, 0);

        telemetry.addLine("Turn saved to: " + filename);
        telemetry.update();
        sleep(2000);
    }

    private List<TurnDataPoint> recordTurnResponse(double targetHeading) {
        List<TurnDataPoint> data = new ArrayList<>();
        ElapsedTime stepTimer = new ElapsedTime();
        stepTimer.reset();

        // Run the control loop and recording for exactly 5 seconds
        while (stepTimer.seconds() < RECORDING_DURATION && opModeIsActive()) {
            double now = stepTimer.seconds();

            // Update Odometry localization
            driveTrain.update();

            // --- SENSOR READS ---
            double currentHeading = driveTrain.getHeading();
            double turnSpeed = driveTrain.getTurnSpeed();

            // --- CONTROL LOGIC ---
            // We use the hardware class's calculation method to ensure we test the exact logic used in auto
            double power = driveTrain.calculateTurnPower(targetHeading, currentHeading);

            // Re-calculate raw error just for logging purposes (to match standard -180 to 180 normalization)
            double error = VexOdometryDriveTrain.normalizeAngle(targetHeading - currentHeading);

            // --- ACTUATOR OUTPUT ---
            // driveTrain.moveRobot(drive, strafe, turn)
            driveTrain.moveRobot(0, 0, power);

            // --- RECORD DATA ---
            data.add(new TurnDataPoint(now, currentHeading, targetHeading, error, turnSpeed, power, CURRENT_TEST_GAIN));
        }

        // Stop
        driveTrain.moveRobot(0, 0, 0);

        return data;
    }

    private void writeAllDataToFile(String filename, List<String> csvLines) {
        File file = new File(AppUtil.FIRST_FOLDER, filename);
        try (FileWriter writer = new FileWriter(file)) {
            // Write Header
            writer.append("Time,CurrentHeading,TargetHeading,Error,TurnSpeed,Power,Gain\n");

            // Write All Lines
            for (String line : csvLines) {
                writer.append(line).append("\n");
            }
            writer.flush();
            telemetry.addLine("File Write Success!");
        } catch (IOException e) {
            telemetry.addLine("Error writing file: " + e.getMessage());
        }
    }
}
