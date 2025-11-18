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

@TeleOp(name = "[Vex] Shooter Spin Recorder", group = "VexTest")
@Disabled
public class VexShooterSpinRecorder extends LinearOpMode {

    // Using a simple data class to hold our measurements
    private static class CalibrationDataPoint {
        final double power;
        final double voltage;
        final double rpm;

        CalibrationDataPoint(double power, double voltage, double rpm) {
            this.power = power;
            this.voltage = voltage;
            this.rpm = rpm;
        }

        @Override
        public String toString() {
            // Returns data in CSV format
            return String.format(Locale.US, "%.2f,%.3f,%.1f", power, voltage, rpm);
        }
    }

    private final VexActuators actuators = new VexActuators(this);
    private final ElapsedTime buttonTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        actuators.init(hardwareMap);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready for calibration.");
        telemetry.addLine("\nPress (A) to start a new calibration run.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Use a timer to prevent multiple 'A' presses from being registered at once
            if (gamepad1.a && buttonTimer.seconds() > 0.5) {
                runCalibrationSequence();
                buttonTimer.reset(); // Reset timer after a run is complete
            }

            telemetry.addLine("Ready for new calibration run.");
            telemetry.addLine("Press (A) to start.");
            telemetry.update();
        }
    }

    private void runCalibrationSequence() {
        telemetry.addLine("Starting calibration run...");
        telemetry.update();
        List<CalibrationDataPoint> dataLog = new ArrayList<>();

        // Get the voltage before the test starts for the filename
        double initialVoltage = actuators.getVoltage();

        // Iterate through power levels from 0.0 to 1.0
        for (double power = 0.3; power <= 1.0; power += 0.05) {
            if (!opModeIsActive()) break; // Exit loop if OpMode is stopped

            // Set the shooter power
            actuators.setShooterPower(power);

            // Display current status on the driver station
            telemetry.addData("Testing Power", "%.2f", power);
            telemetry.update();

            // Wait for the motors to stabilize at the new power level
            sleep(3000);

            // Record the measurements
            double currentVoltage = actuators.getVoltage();
            double currentRpm = actuators.getShooterRPM();
            dataLog.add(new CalibrationDataPoint(power, currentVoltage, currentRpm));

            // Display the recorded data immediately
            telemetry.addData("Testing Power", "%.2f", power);
            telemetry.addData("Voltage", "%.3f", currentVoltage);
            telemetry.addData("RPM", "%.1f", currentRpm);
            telemetry.update();
        }

        // Turn off the shooter motors after the run
        actuators.setShooterPower(0);

        // Write the collected data to a new file
        writeLogsToFile(dataLog, initialVoltage);

        telemetry.addLine("Calibration run complete.");
        telemetry.update();
        sleep(1000); // Brief pause before allowing another run
    }

    private void writeLogsToFile(List<CalibrationDataPoint> dataLog, double initialVoltage) {
        if (dataLog.isEmpty()) {
            return; // Nothing to write
        }

        try {
            // Create a filename with a timestamp and initial voltage
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());
            String voltageString = String.format(Locale.US, "%.2fV", initialVoltage);
            String filename = "ShooterCalibration_" + timestamp + "_" + voltageString + ".csv";
            File file = new File(AppUtil.FIRST_FOLDER, filename);

            telemetry.clearAll(); // Clear previous telemetry
            telemetry.addLine("Writing to file: " + filename);
            telemetry.update();
            sleep(500); // give user time to read

            // Write data to the CSV file
            try (FileWriter writer = new FileWriter(file)) {
                // Write header row
                String header = "Power,Voltage,RPM";
                writer.append(header).append("\n");
                telemetry.addLine(header);
                telemetry.update();

                // Write each data point
                for (CalibrationDataPoint dataPoint : dataLog) {
                    String line = dataPoint.toString();
                    writer.append(line).append("\n");
                    telemetry.addLine(line); // Also display the line on telemetry
                    telemetry.update();
                }
                writer.flush();
            }

            // Notify user that the file has been saved
            telemetry.addLine("\nSuccessfully saved data to:");
            telemetry.addLine(filename);

        } catch (IOException e) {
            telemetry.addLine("Error writing to file: " + e.getMessage());
        } finally {
            telemetry.update();
            sleep(2000); // Display final message for 2 seconds
        }
    }
}
