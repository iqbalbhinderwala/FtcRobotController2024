package org.firstinspires.ftc.teamcode.Vex.Calibration;

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

@TeleOp(name = "[Vex] Ball Shooting Recorder", group = "VexTest")
public class VexBallShootingRecorder extends LinearOpMode {
    // --- Data Class ---
    private static class RpmDataPoint {
        final double timeSeconds;
        final double power;
        final double voltage;
        final double rpm;
        final double mrpm;

        RpmDataPoint(double time, double power, double voltage, double rpm) {
            this.timeSeconds = time;
            this.power = power;
            this.voltage = voltage;
            this.rpm = rpm;
            this.mrpm = rpm / 1000.0;
        }

        @Override
        public String toString() {
            // CSV Format: Time,Power,Voltage,RPM,mRPM
            return String.format(Locale.US, "%.4f,%.3f,%.3f,%.1f,%.3f", timeSeconds, power, voltage, rpm, mrpm);
        }
    }

    // --- Hardware & State ---
    private final VexActuators actuators = new VexActuators(this);
    private final ElapsedTime buttonDebounceTimer = new ElapsedTime();
    private final List<RpmDataPoint> recordedData = new ArrayList<>();

    // --- Configuration ---
    private final double CLOSR_RPM = 2300;
    private final double FAR_RPM = 3100;

    // Default selection
    private double selectedTargetRpm = CLOSR_RPM;

    // Sequence Timings
    private final double GATE_CLOSED_WAIT_SEC = 0.5;
    private final double SHOOTING_DURATION_SEC = 3.0;

    // Session State
    private int runIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        actuators.init(hardwareMap);
        telemetry.setMsTransmissionInterval(50);
        buttonDebounceTimer.reset();

        // --- INIT PHASE: Select Target RPM ---
        while (!isStarted() && !isStopRequested()) {
            // Use X vs B to choose target
            if (gamepad1.x) {
                selectedTargetRpm = CLOSR_RPM;
            } else if (gamepad1.b) {
                selectedTargetRpm = FAR_RPM;
            }

            displayMenu();
        }

        if (isStopRequested()) return;

        // --- ACTIVE PHASE: Run Sessions ---
        while (opModeIsActive() && !isStopRequested()) {
            displayMenu();

            // Check input (Debounced)
            if (buttonDebounceTimer.seconds() > 0.5) {
                // Only press A to move to the next run index
                if (gamepad1.a) {
                    runTestSequence(selectedTargetRpm);
                    buttonDebounceTimer.reset();
                }
            }
        }

        // Safety stop at end
        actuators.setShooterPower(0);
    }

    /**
     * Executes the shooting sequence:
     * 1. Close Gate -> Wait
     * 2. Spin Up (1.0 Power) -> Wait for RPM
     * 3. Set Holding Power -> Open Gate
     * 4. Record for 3 seconds -> Stop -> Save
     */
    private void runTestSequence(double targetRpm) {
        telemetry.clearAll();
        telemetry.addLine(String.format(Locale.US, "RUN %d STARTING...", runIndex));
        telemetry.update();

        // 1. Close gate and wait
        actuators.closeGateA();
        sleep((long)(GATE_CLOSED_WAIT_SEC * 1000));

        // 2. Setup Recording
        ElapsedTime recordingTimer = new ElapsedTime(); // Starts at 0.0
        recordedData.clear();

        // 3. Spin up phase (Max Power)
        double predictMaintainPower = 1.0;
        actuators.setShooterPower(predictMaintainPower);

        // Wait until RPM >= Target (but keep recording)
        while (opModeIsActive() && actuators.getShooterRPM() < targetRpm) {
            double now = recordingTimer.seconds();
            double volts = actuators.getVoltage();
            double rpm = actuators.getShooterRPM();

            recordedData.add(new RpmDataPoint(now, predictMaintainPower, volts, rpm));

            telemetry.addData("Run", runIndex);
            telemetry.addData("Phase", "Spin Up");
            telemetry.addData("Target", "%.0f", targetRpm);
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.update();
        }

        if (!opModeIsActive()) return;

        // 4. Target Reached: Switch to predictive power & Open Gate
        predictMaintainPower = actuators.predictShooterPowerForTargetRPM(targetRpm);
        actuators.setShooterPower(predictMaintainPower);

        actuators.openGateA();

        // 5. Shooting Phase (Wait 3 seconds while recording)
        ElapsedTime shootingTimer = new ElapsedTime();

        while (opModeIsActive() && shootingTimer.seconds() < SHOOTING_DURATION_SEC) {
            double now = recordingTimer.seconds();
            double volts = actuators.getVoltage();
            double rpm = actuators.getShooterRPM();

            recordedData.add(new RpmDataPoint(now, predictMaintainPower, volts, rpm));

            telemetry.addData("Run", runIndex);
            telemetry.addData("Phase", "Shooting / Open Gate");
            telemetry.addData("Time Left", "%.1fs", SHOOTING_DURATION_SEC - shootingTimer.seconds());
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.update();
        }

        // 6. Stop
        actuators.setShooterPower(0);

        // 7. Save
        telemetry.addLine("Saving Data...");
        telemetry.update();
        saveRecordingToFile(targetRpm);

        // Increment index for next run
        runIndex++;

        sleep(2000); // Brief pause before returning to menu
        actuators.closeGateA();
    }

    private void saveRecordingToFile(double targetRpm) {
        if (recordedData.isEmpty()) return;

        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());

        // Format: ShooterTune_RunX_TargetY_Timestamp.csv
        String filename = String.format(Locale.US, "ShooterTune_Run%d_Target%.0f_%s.csv",
                runIndex, targetRpm, timestamp);

        File file = new File(AppUtil.FIRST_FOLDER, filename);

        try (FileWriter writer = new FileWriter(file)) {
            writer.append("RunIndex,Time,Power,Voltage,RPM,mRPM\n");

            for (RpmDataPoint point : recordedData) {
                // Prepend RunIndex to every line for easy concatenation later
                writer.append(String.format(Locale.US, "%d,%s\n", runIndex, point.toString()));
            }

            writer.flush();
            telemetry.addLine("Saved: " + filename);
        } catch (IOException e) {
            telemetry.addLine("Error saving: " + e.getMessage());
        }
        telemetry.update();
    }

    private void displayMenu() {
        telemetry.clearAll();
        telemetry.addLine("--- Vex Shooter RPM Tuner ---");

        // Show RPM Selection State
        telemetry.addLine("\nTARGET SELECTION:");
        String lowMark = (selectedTargetRpm == CLOSR_RPM) ? ">> " : "   ";
        String highMark = (selectedTargetRpm == FAR_RPM) ? ">> " : "   ";

        telemetry.addData(lowMark + "[X]", "Low RPM (%.0f)", CLOSR_RPM);
        telemetry.addData(highMark + "[B]", "High RPM (%.0f)", FAR_RPM);

        telemetry.addLine("\n---------------------------");

        if (!isStarted()) {
            telemetry.addLine("Select RPM with X / B.");
            telemetry.addLine("Press START when ready.");
        } else {
            telemetry.addData("NEXT RUN INDEX", runIndex);

            // Instructions based on index
            String ballCount = (runIndex <= 3) ? String.valueOf(runIndex) : "3+";
            telemetry.addData("ACTION", "Load %s balls.", ballCount);

            telemetry.addLine("\nCONTROLS:");
            telemetry.addLine("Press [A] to RECORD RUN.");
        }

        telemetry.addLine("\nSTATUS:");
        telemetry.addData("Voltage", "%.2f V", actuators.getVoltage());
        telemetry.update();
    }
}
