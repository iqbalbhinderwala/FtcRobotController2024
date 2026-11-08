package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Hardware.Defender;
import org.firstinspires.ftc.teamcode.John.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.John.Hardware.StatusLights;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Finds the defender servo's STOWED and DEPLOYED positions (Defender), tests shooting with the brace
 * at different angles (does it get in the way of the balls?), and checks the LED wiring
 * (StatusLights). The robot does not drive. Nothing moves until a button is pressed after START.
 *
 * SERVO mode (gamepad1):
 *   dpad up / down     position +/- 0.01        dpad right / left  +/- 0.05
 *   right / left bumper  +/- 0.002 (fine)
 *   A held  SHOOT: flywheel at SHOT_RPM, then a volley of BALLS + 1 indexer steps; let go: stop
 *   B  toggle between STOWED and DEPLOYED
 *   X  save as STOWED   Y  save as DEPLOYED
 *   Start  EXTENDED_PWM on / off (wider travel for servos that need it)
 *   Right trigger held  SWEEP between 0.3 and 0.7 every second (is it wired and alive?)
 * Back switches to LED mode: A / B / X / Y toggle ports 0 / 1 / 2 / 3; note the color each lights.
 *
 * Saved values last until the app restarts: copy them into Defender.java (telemetry shows them).
 * Each volley is logged with the brace position to /sdcard/FIRST/john_logs/r<run>_john_defender_shots_<time>.csv.
 */
@Config("JohnDefenderCalibration")
@TeleOp(name = "[John] Defender Calibration", group = "JohnTest")
public class DefenderCalibration extends LinearOpMode {

    public static double SHOT_RPM = 2275;
    public static int BALLS = 4;

    private static final String LOG_HEADER = "volley,t_s,defender_pos,rpm_set,rpm_at_balls,steps_done,event";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Defender defender = new Defender(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        StatusLights lights = new StatusLights(hardwareMap);
        CsvLog log = new CsvLog("john_defender_shots", LOG_HEADER);
        lights.off();

        while (opModeInInit()) {
            telemetry.addLine("Nothing moves until a button after START.");
            telemetry.addLine("Brace folded in by hand first.");
            telemetry.addData("Saved", "stow %s, deploy %s", fmt(Defender.STOWED_POSITION), fmt(Defender.DEPLOYED_POSITION));
            telemetry.addLine("First dpad press: servo to " + (Double.isNaN(Defender.STOWED_POSITION) ? "0.5" : "stowed"));
            telemetry.update();
        }
        gamepad1.resetEdgeDetection();
        ElapsedTime clock = new ElapsedTime();
        ElapsedTime sweep = new ElapsedTime();
        boolean ledMode = false;
        boolean[] on = new boolean[4];
        // Nothing saved yet: dpad steps start from the middle (the first press moves the servo there)
        double target = Double.isNaN(Defender.STOWED_POSITION) ? 0.5 : Defender.STOWED_POSITION;
        String note = "";
        boolean driven = false;
        // Shooting
        boolean aLatched = false;   // after a volley, A must be let go before the next
        boolean volleyOn = false;
        int volley = 0;
        int stepsAtStart = 0, lastSteps = 0;
        List<Long> rpmAtBalls = new ArrayList<>();
        List<String> history = new ArrayList<>();

        while (opModeIsActive()) {
            String event = null;
            if (gamepad1.backWasPressed()) {
                ledMode = !ledMode;
            }
            if (ledMode) {
                boolean[] press = {gamepad1.aWasPressed(), gamepad1.bWasPressed(), gamepad1.xWasPressed(), gamepad1.yWasPressed()};
                for (int i = 0; i < 4; i++) {
                    if (press[i]) {
                        on[i] = !on[i];
                        lights.channel(i, on[i]);
                    }
                }
                telemetry.addLine(">>> LED TEST (Back: servo)");
                telemetry.addLine("A port 0 | B port 1 | X port 2 | Y port 3");
                for (int i = 0; i < 4; i++) {
                    telemetry.addData("Port " + i, on[i] ? "ON" : "off");
                }
                telemetry.addLine("Note the color of each port");
                telemetry.addData("Lower = red?", StatusLights.LOWER_PORT_IS_RED);
            } else {
                // ---- Brace position ----
                double step = 0;
                if (gamepad1.dpadUpWasPressed()) step = 0.01;
                if (gamepad1.dpadDownWasPressed()) step = -0.01;
                if (gamepad1.dpadRightWasPressed()) step = 0.05;
                if (gamepad1.dpadLeftWasPressed()) step = -0.05;
                if (gamepad1.rightBumperWasPressed()) step = 0.002;
                if (gamepad1.leftBumperWasPressed()) step = -0.002;
                if (step != 0) {
                    target = Math.max(0.0, Math.min(1.0, target + step));
                    defender.setPosition(target);
                    driven = true;
                }
                if (gamepad1.xWasPressed()) Defender.STOWED_POSITION = target;
                if (gamepad1.yWasPressed()) Defender.DEPLOYED_POSITION = target;
                if (gamepad1.bWasPressed()) {
                    if (!Defender.calibrated()) {
                        note = "B: save STOWED (X) and DEPLOYED (Y) first";
                    } else {
                        boolean toDeployed = Math.abs(target - Defender.DEPLOYED_POSITION)
                                > Math.abs(target - Defender.STOWED_POSITION);
                        if (toDeployed) defender.deploy(); else defender.stow();
                        target = toDeployed ? Defender.DEPLOYED_POSITION : Defender.STOWED_POSITION;
                        driven = true;
                        note = "";
                    }
                }
                if (gamepad1.right_trigger > 0.5) {
                    // Wiring check: a big, slow back-and-forth
                    target = (sweep.seconds() % 2.0) < 1.0 ? 0.3 : 0.7;
                    defender.setPosition(target);
                    driven = true;
                }
                if (gamepad1.startWasPressed()) {
                    Defender.EXTENDED_PWM = !Defender.EXTENDED_PWM;
                    defender.applyPwmRange();
                }

                // ---- Shooting: A held ----
                if (!gamepad1.a) {
                    aLatched = false;
                    if (shooter.firing()) {
                        shooter.cancel();
                        event = "volley cancelled: A let go";
                    }
                }
                if (gamepad1.a && !aLatched && !shooter.firing()) {
                    aLatched = true;
                    volleyOn = true;
                    volley++;
                    stepsAtStart = shooter.indexer().stepCount();
                    lastSteps = stepsAtStart;
                    rpmAtBalls.clear();
                    shooter.startVolley(BALLS + 1);
                    event = String.format(Locale.US, "volley %d at %.0f RPM, defender %s", volley, SHOT_RPM,
                            driven ? String.format(Locale.US, "%.3f", target) : "not driven");
                }
                int steps = shooter.indexer().stepCount();
                if (volleyOn && steps != lastSteps) {
                    lastSteps = steps;
                    rpmAtBalls.add(Math.round(shooter.flywheel().rpm()));
                }
                if (volleyOn && !shooter.firing()) {
                    volleyOn = false;
                    int done = Math.abs(shooter.indexer().stepCount() - stepsAtStart);
                    log.row(volley, clock.seconds(), driven ? target : null, SHOT_RPM, join(rpmAtBalls), done, event);
                    log.flush();
                    history.add(0, String.format(Locale.US, "#%d brace %s: %d steps", volley,
                            driven ? String.format(Locale.US, "%.3f", target) : "-", done));
                    while (history.size() > 4) history.remove(history.size() - 1);
                }

                telemetry.addLine(shooter.firing() ? ">>> FIRING (let go of A: stop)" : ">>> DEFENDER (Back: LEDs)");
                if (!note.isEmpty()) {
                    telemetry.addLine("!!! " + note);
                }
                telemetry.addData("Brace", driven ? String.format(Locale.US, "%.3f", target) : "not driven yet");
                telemetry.addData("Stowed", "%s (X saves)", fmt(Defender.STOWED_POSITION));
                telemetry.addData("Deployed", "%s (Y saves)", fmt(Defender.DEPLOYED_POSITION));
                telemetry.addData("Flywheel", "%.0f / %.0f", shooter.flywheel().rpm(), SHOT_RPM);
                for (String h : history) {
                    telemetry.addLine(h);
                }
                telemetry.addLine("A hold shoot | B stow/deploy");
                telemetry.addLine("dpad +-0.01/0.05 | bumpers +-0.002");
                telemetry.addLine("X/Y save | RT sweep | Start ext PWM");
                telemetry.addData("Port", defender.diagnostics());
            }

            // The shooter runs in both modes (a volley finishes even if Back is pressed)
            shooter.setRpmOverride(SHOT_RPM);
            shooter.setDistance(Double.NaN);   // applies the speed; near fire tolerance
            shooter.setSpin(gamepad1.a && !ledMode);
            String shooterEvent = shooter.update();
            if (shooterEvent != null) {
                event = event == null ? shooterEvent : event + "; " + shooterEvent;
            }
            telemetry.update();
        }
        shooter.stop();
        lights.off();
        log.close();
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "not saved" : String.format(Locale.US, "%.3f", v);
    }

    private static String join(List<Long> values) {
        StringBuilder sb = new StringBuilder();
        for (Long v : values) {
            if (sb.length() > 0) sb.append(' ');
            sb.append(v);
        }
        return sb.toString();
    }
}
