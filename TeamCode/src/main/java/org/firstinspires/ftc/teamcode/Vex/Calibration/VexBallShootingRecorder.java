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
    }

    // --- Hardware & State ---
    private final VexActuators actuators = new VexActuators(this);
    private final ElapsedTime buttonDebounceTimer = new ElapsedTime();
    private final List<RpmDataPoint> recordedData = new ArrayList<>();

    // --- Configuration ---
    private final double CLOSE_RPM = 2300;
    private final double FAR_RPM = 3100;

    // Default selection
    private double selectedTargetRpm = CLOSE_RPM;

    // Sequence Timings
    private final double GATE_CLOSED_WAIT_SEC = 0.5;
    private final double SHOOTING_DURATION_SEC = 4.0;

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
                selectedTargetRpm = CLOSE_RPM;
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
     * 4. Record for 7 seconds -> Stop -> Save
     */
    private void runTestSequence(double targetRpm) {
        telemetry.clearAll();
        telemetry.addLine(String.format(Locale.US, "RUN %d STARTING...", runIndex));
        telemetry.update();

        // CAPTURE RESTING VOLTAGE (before motors start)
        double restingVoltage = actuators.getVoltage();

        // 1. Close gate and wait
        actuators.closeGateA();
        sleep((long)(GATE_CLOSED_WAIT_SEC * 1000));

        // 2. Setup Recording
        ElapsedTime recordingTimer = new ElapsedTime(); // Starts at 0.0
        recordedData.clear();

        // 3. Spin up phase (Max Power)
        actuators.setShooterPower(1.0);
        actuators.setIntakePower(1.0);

        // Wait until RPM >= Target (but keep recording)
        while (opModeIsActive() && actuators.getShooterRPM() < targetRpm - actuators.SHOOTER_RPM_INCREMENT ) {
            double now = recordingTimer.seconds();
            double volts = actuators.getVoltage();
            double rpm = actuators.getShooterRPM();
            double power = actuators.getShooterPower();

            recordedData.add(new RpmDataPoint(now, power, volts, rpm));

            telemetry.addData("Run", runIndex);
            telemetry.addData("Phase", "Spin Up");
            telemetry.addData("Target", "%.0f", targetRpm);
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        }

        if (!opModeIsActive()) return;

        // 4. Target Reached: Switch to predictive power & Open Gate
        actuators.setShooterPower(actuators.predictShooterPowerForTargetRPM(targetRpm));

        actuators.openGateA();

        // 5. Shooting Phase (Wait SHOOTING_DURATION_SEC while recording)
        ElapsedTime shootingTimer = new ElapsedTime();

        while (opModeIsActive() && shootingTimer.seconds() < SHOOTING_DURATION_SEC) {
            if (actuators.getShooterRPM() < targetRpm - actuators.SHOOTER_RPM_INCREMENT ) {
                actuators.setShooterPower(1.0);
            } else {
                actuators.setShooterPower(actuators.predictShooterPowerForTargetRPM(targetRpm));
            }

            double now = recordingTimer.seconds();
            double volts = actuators.getVoltage();
            double rpm = actuators.getShooterRPM();
            double power = actuators.getShooterPower();

            recordedData.add(new RpmDataPoint(now, power, volts, rpm));

            telemetry.addData("Run", runIndex);
            telemetry.addData("Phase", "Shooting / Open Gate");
            telemetry.addData("Time Left", "%.1fs", SHOOTING_DURATION_SEC - shootingTimer.seconds());
            telemetry.addData("Target", "%.0f", targetRpm);
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        }

        // 6. Stop
        actuators.setShooterPower(0);
        actuators.setIntakePower(0);

        // 7. Save
        telemetry.addLine("Saving Data...");
        telemetry.update();
        saveRecordingToFile(targetRpm, restingVoltage);

        // Increment index for next run
        runIndex++;

        sleep(2000); // Brief pause before returning to menu
        actuators.closeGateA();
    }

    private void saveRecordingToFile(double targetRpm, double restingVoltage) {
        if (recordedData.isEmpty()) return;

        // Format: ShooterTune_Timestamp_RunX_TargetY_Z.zzV.csv
        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());
        String filename = String.format(Locale.US, "ShooterTune_%s_Run%d_Target%.0f_%.2fV.csv",
                timestamp, runIndex, targetRpm, restingVoltage);

        File file = new File(AppUtil.FIRST_FOLDER, filename);

        try (FileWriter writer = new FileWriter(file)) {
            // Write the custom header order
            writer.append("RunIndex,Time,mRPM,Power,Voltage,RPM\n");

            for (RpmDataPoint point : recordedData) {
                // Manually format the columns instead of using point.toString()
                writer.append(String.format(Locale.US, "%d,%.4f,%.3f,%.3f,%.3f,%.1f\n",
                        runIndex,
                        point.timeSeconds,
                        point.mrpm,
                        point.power,
                        point.voltage,
                        point.rpm));
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
        String lowMark = (selectedTargetRpm == CLOSE_RPM) ? ">> " : "   ";
        String highMark = (selectedTargetRpm == FAR_RPM) ? ">> " : "   ";

        telemetry.addData(lowMark + "[X]", "Low RPM (%.0f)", CLOSE_RPM);
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
