package org.firstinspires.ftc.teamcode.Vex.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexActuators;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "[Vex] Shooter Step Response Recorder", group = "VexTest")
//@Disabled
public class VexShooterStepResponseRecorder extends LinearOpMode {

    // Data class for high-frequency time sampling
    private static class TimeSeriesDataPoint {
        final double timeSeconds;
        final double targetPower;
        final double voltage;
        final double rpm;

        TimeSeriesDataPoint(double timeSeconds, double targetPower, double voltage, double rpm) {
            this.timeSeconds = timeSeconds;
            this.targetPower = targetPower;
            this.voltage = voltage;
            this.rpm = rpm;
        }

        @Override
        public String toString() {
            // CSV Format: Time, Power, Voltage, RPM
            return String.format(Locale.US, "%.4f,%.2f,%.3f,%.1f", timeSeconds, targetPower, voltage, rpm);
        }
    }

    private final VexActuators actuators = new VexActuators(this);
    private final ElapsedTime buttonTimer = new ElapsedTime();

    // Configuration constants
    private static final double RECORDING_DURATION = 10.0; // Seconds to record each step
    private static final double START_POWER = 0.5;
    private static final double END_POWER = 1.0;
    private static final double STEP_POWER = 0.1;

    @Override
    public void runOpMode() {
        actuators.init(hardwareMap);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready for Step Response Calibration.");
        telemetry.addLine("This will record RPM over time for 0 -> Target jumps.");
        telemetry.addLine("\nPress (A) to start the full sequence.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a && buttonTimer.seconds() > 1.0) {
                runFullStepSequence();
                buttonTimer.reset();
            }

            telemetry.addLine("Ready.");
            telemetry.addLine("Press (A) to start sequence.");
            telemetry.addData("Current Voltage", "%.2f V", actuators.getVoltage());
            telemetry.update();
        }
    }

    private void runFullStepSequence() {
        String timestampBase = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());

        // Loop through powers: 0.5, 0.6, ..., 1.0
        for (double targetPower = START_POWER; targetPower <= END_POWER + 0.01; targetPower += STEP_POWER) {
            if (!opModeIsActive()) break;

            // 1. Reset Phase: Ensure motor is stopped completely before the step
            telemetry.addLine("Cooling down / Stopping...");
            telemetry.update();
            actuators.setShooterPower(0);
            sleep(7000); // Wait for wheel to actually stop spinning

            // Get voltage after stopping, before the next step starts
            double restingVoltage = actuators.getVoltage();

            // 2. Recording Phase
            telemetry.addData("Testing Step", "0 -> %.2f", targetPower);
            telemetry.addData("Resting Voltage", "%.2fV", restingVoltage);
            telemetry.update();

            List<TimeSeriesDataPoint> stepData = recordStepResponse(targetPower);

            // 3. Write Data Phase (Write to individual file for this step)
            // Generate unique filename for this specific power step
            String filename = String.format(Locale.US, "ShooterStep_P%.2f_%.1fV_%s.csv",
                    targetPower, restingVoltage, timestampBase);

            List<String> csvLines = new ArrayList<>();
            for (TimeSeriesDataPoint point : stepData) {
                csvLines.add(point.toString());
            }

            writeAllDataToFile(filename, csvLines);

            telemetry.addLine("Step saved to: " + filename);
            telemetry.update();
        }

        actuators.setShooterPower(0);
        telemetry.addLine("Full Sequence Complete.");
        telemetry.update();
        sleep(5000);
    }

    private List<TimeSeriesDataPoint> recordStepResponse(double targetPower) {
        List<TimeSeriesDataPoint> data = new ArrayList<>();
        ElapsedTime stepTimer = new ElapsedTime();

        // Turn on the motor
        actuators.setShooterPower(targetPower);
        stepTimer.reset();

        // Record as fast as possible for the duration
        while (stepTimer.seconds() < RECORDING_DURATION && opModeIsActive()) {
            double now = stepTimer.seconds();
            double volts = actuators.getVoltage();
            double rpm = actuators.getShooterRPM();

            data.add(new TimeSeriesDataPoint(now, targetPower, volts, rpm));

            // Optional: Provide visual feedback every ~200ms without slowing down loop too much
            if (data.size() % 20 == 0) {
                telemetry.addData("Recording...", "%.2fs / %.1fs", now, RECORDING_DURATION);
                telemetry.addData("RPM", "%.1f", rpm);
                telemetry.update();
            }
        }

        return data;
    }

    private void writeAllDataToFile(String filename, List<String> csvLines) {
        File file = new File(AppUtil.FIRST_FOLDER, filename);
        try (FileWriter writer = new FileWriter(file)) {
            // Write Header (RunID removed)
            writer.append("Time,TargetPower,Voltage,RPM\n");

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
