package org.firstinspires.ftc.teamcode.John.Test;

import static com.pedropathing.utils.Utils.quadraticFit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.JohnTuning;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Pedro's forward / strafe braking coefficients, measured the way Pedro's Foresight tuner does it,
 * but without stalling at low power (2026-11-13: John v2 did not strafe at all at the tuner's lowest
 * power, 0.3, so the trial had to be finished by hand).
 *
 * As the tuner (ForwardBraking / StrafeBraking): drive along the axis at a power, alternately away
 * from the start (until DISTANCE_IN out) and back (until within RETURN_IN of it); there cut the power
 * (a tiny reverse power, motors floating, the heading held) and measure how far John slides until
 * he is slower than STOPPED_IPS. Then fit slide distance = a v + b v^2 over the trials with Pedro's
 * own quadraticFit: a, b are linearBrakeCoefficients / quadraticBrakeCoefficients (per axis).
 *
 * The difference: if John is still slower than STALL_IPS STALL_MS into a trial, the power rises by
 * RESCUE_STEP every RESCUE_MS until he moves. The fit only uses the speed when the brake starts, not
 * how he got to it, so a rescued trial is as good as any other.
 *
 * Axis: chosen with the dpad during INIT (up forward, left / right strafe, down both: forward, then
 * a pause to turn John 90 deg, A to go on). AXES is only the starting choice.
 * Room: one axis at a time. With DISTANCE_IN 60
 * and RETURN_IN 12 a run spans about 15" behind the start to 85" ahead of it (the slides at full speed
 * are up to ~25" forward, ~10" strafe): 6 tiles (144") along the axis, John starting about 2 ft from
 * one end. With only 60" use DISTANCE_IN 30 (start ~8" from the end). He goes first toward intake B
 * (forward) or toward the shooter side (strafe).
 * B aborts. The fit is shown at the end; copy it into Pedro/Constants.java.
 * Logged to /sdcard/FIRST/john_v2_logs/r<run>_john_brake_cal_<time>.csv (one row per trial).
 */
@Config("JohnBrakeCalibration")
@TeleOp(name = "[John] Brake Calibration", group = "JohnTest")
public class BrakeCalibration extends LinearOpMode {

    /** Starting choice (the dpad in INIT changes it): "forward", "strafe" or "forward,strafe". */
    public static String AXES = "forward";
    /** Trial powers, high to low (the tuner: 5 from 1.0 to 0.3, more at the high end). */
    public static String POWERS = "1.0,0.85,0.7,0.55,0.4,0.3";
    /** Out: brake once this far from the start. Back: brake within RETURN_IN of the start (the
     *  return slide then ends near the start instead of far behind it). */
    public static double DISTANCE_IN = 60;
    public static double RETURN_IN = 12;
    public static double STOPPED_IPS = 0.25;
    public static double MAX_BRAKE_S = 7.0;
    public static double IDLE_S = 1.0;
    /** The tuner's braking command: a tiny power against the motion. */
    public static double BRAKING_POWER = 0.001;
    // Stall rescue
    public static double STALL_IPS = 3.0;
    public static double STALL_MS = 400;
    public static double RESCUE_STEP = 0.05;
    public static double RESCUE_MS = 200;

    private static final String LOG_HEADER = "t_s,axis,trial,direction,power_set,power_used,rescued,"
            + "v_at_brake_ips,slide_in,brake_s,heading_deg";

    private enum State { DRIVE, BRAKE, WAIT, DONE }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        CsvLog log = new CsvLog("john_brake_cal", LOG_HEADER);
        double[] powers = parse(POWERS);
        String axesChoice = AXES.replace(" ", "");

        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) axesChoice = "forward";
            if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) axesChoice = "strafe";
            if (gamepad1.dpadDownWasPressed()) axesChoice = "forward,strafe";
            telemetry.addLine("John drives on his own: clear floor!");
            telemetry.addData("Axis", "%s   (dpad: up forward, left/right strafe, down both)",
                    axesChoice.toUpperCase(Locale.US));
            telemetry.addData("Room", "about %.0f in along the axis; start ~%.0f in from one end", DISTANCE_IN + 40, DISTANCE_IN > 40 ? 24.0 : 8.0);
            telemetry.addLine("First move: forward = toward intake B, strafe = toward the shooter side");
            telemetry.addData("Trials", "%d per axis, powers %s", powers.length, POWERS);
            telemetry.addLine("B during the run: abort");
            telemetry.update();
        }
        ElapsedTime clock = new ElapsedTime();
        List<String> summary = new ArrayList<>();
        String[] axes = axesChoice.split(",");

        for (int axisIndex = 0; axisIndex < axes.length; axisIndex++) {
            boolean strafe = axes[axisIndex].trim().equals("strafe");
            // Between axes: stop, let the driver turn John 90 deg on the clear strip, A to go on
            if (axisIndex > 0) {
                gamepad1.resetEdgeDetection();
                while (opModeIsActive() && !gamepad1.aWasPressed() && !gamepad1.b) {
                    drivetrain.stop(false);
                    telemetry.addLine(">>> Turn John 90 deg for the " + (strafe ? "STRAFE" : "FORWARD") + " test, then press A");
                    telemetry.addLine(strafe ? "He goes toward the shooter side first, ~2 ft from the end"
                            : "He goes toward intake B first, ~2 ft from the end");
                    for (String line : summary) telemetry.addLine(line);
                    telemetry.addLine("B: stop here");
                    telemetry.update();
                }
                if (!opModeIsActive() || gamepad1.b) break;
            }
            localizer.setPose(Pose.zero());
            localizer.update();
            List<double[]> data = new ArrayList<>();
            State state = State.DRIVE;
            int trial = 0;
            int direction = 1;
            double power = powers[0];
            boolean rescued = false;
            ElapsedTime timer = new ElapsedTime();
            ElapsedTime rescueTimer = new ElapsedTime();
            Pose start = null;
            double vAtBrake = 0;
            List<String> rows = new ArrayList<>();

            while (opModeIsActive() && state != State.DONE) {
                if (gamepad1.b) {
                    drivetrain.stop(false);
                    state = State.DONE;
                    rows.add("ABORTED");
                    break;
                }
                localizer.update();
                Pose p = localizer.pose();
                double along = strafe ? p.y() : p.x();
                double speed = localizer.velocity().toVector2D().magnitude();
                double headingPower = headingPower(localizer);

                switch (state) {
                    case DRIVE: {
                        boolean reached = direction > 0 ? Math.abs(along) > DISTANCE_IN : Math.abs(along) <= RETURN_IN;
                        if (reached) {
                            start = p;
                            vAtBrake = speed;
                            state = State.BRAKE;
                            timer.reset();
                            brake(drivetrain, strafe, direction, headingPower);
                            break;
                        }
                        // Stall rescue: not moving STALL_MS into the trial, raise the power step by step
                        if (timer.milliseconds() > STALL_MS && speed < STALL_IPS && rescueTimer.milliseconds() > RESCUE_MS
                                && power < 1.0) {
                            power = Math.min(1.0, power + RESCUE_STEP);
                            rescued = true;
                            rescueTimer.reset();
                        }
                        double d = power * direction;
                        drivetrain.drive(strafe ? new DrivePowers(0, d, headingPower) : new DrivePowers(d, 0, headingPower), false);
                        break;
                    }
                    case BRAKE: {
                        if (speed > STOPPED_IPS && timer.seconds() < MAX_BRAKE_S) {
                            brake(drivetrain, strafe, direction, headingPower);
                            break;
                        }
                        double slide = Math.hypot(p.x() - start.x(), p.y() - start.y());
                        data.add(new double[]{vAtBrake, slide});
                        log.row(clock.seconds(), strafe ? "strafe" : "forward", trial + 1, direction, powers[Math.min(trial, powers.length - 1)],
                                power, rescued, vAtBrake, slide, timer.seconds(), Math.toDegrees(p.heading()));
                        log.flush();
                        rows.add(String.format(Locale.US, "%d: %.0f in/s -> %.1f in%s", trial + 1, vAtBrake, slide,
                                rescued ? String.format(Locale.US, " (power raised to %.2f)", power) : ""));
                        trial++;
                        if (trial >= powers.length) {
                            drivetrain.stop(false);
                            state = State.DONE;
                        } else {
                            state = State.WAIT;
                            timer.reset();
                        }
                        break;
                    }
                    case WAIT: {
                        drivetrain.stop(false);
                        if (timer.seconds() > IDLE_S) {
                            direction = trial % 2 == 0 ? 1 : -1;
                            power = powers[trial];
                            rescued = false;
                            state = State.DRIVE;
                            timer.reset();
                            rescueTimer.reset();
                        }
                        break;
                    }
                    default:
                        break;
                }

                telemetry.addLine(String.format(Locale.US, ">>> %s braking: trial %d of %d, %s", strafe ? "STRAFE" : "FORWARD",
                        Math.min(trial + 1, powers.length), powers.length, state));
                telemetry.addData("Power", "%.2f%s", power, rescued ? " (raised: was stuck)" : "");
                telemetry.addData("Along / speed", "%.1f in, %.0f in/s", along, speed);
                for (String r : rows) telemetry.addLine(r);
                telemetry.addLine("B: abort");
                telemetry.update();
            }
            drivetrain.stop(false);
            if (data.size() >= 3) {
                double[] c = quadraticFit(data);
                summary.add(String.format(Locale.US, "%s: linear %.6f, quadratic %.7f (%d trials)", strafe ? "STRAFE" : "FORWARD",
                        c[0], c[1], data.size()));
                log.row(clock.seconds(), strafe ? "strafe" : "forward", "fit", null, null, null, null, c[0], c[1], null, null);
                log.flush();
            } else {
                summary.add((strafe ? "STRAFE" : "FORWARD") + ": not enough trials (" + data.size() + ")");
            }
            if (!opModeIsActive() || gamepad1.b) break;
        }

        // Results: copy into Constants.java (linearBrakeCoefficients / quadraticBrakeCoefficients diag(forward, strafe))
        while (opModeIsActive()) {
            drivetrain.stop(false);
            telemetry.addLine(">>> DONE: copy into Pedro/Constants.java");
            for (String s : summary) telemetry.addLine(s);
            telemetry.addLine("Constants now: forward / strafe");
            telemetry.addLine("  linear diag(forward, strafe), quadratic diag(...)");
            telemetry.update();
        }
        log.close();
    }

    /** The tuner's heading hold: back to the start heading, with the heading brake prediction. */
    private static double headingPower(Localizer localizer) {
        double w = localizer.velocity().omega;
        double brakeDist = JohnTuning.HEADING_BRAKE_A * w + JohnTuning.HEADING_BRAKE_B * w * w * Math.signum(w);
        double error = Angle.normalizeSigned(-localizer.pose().heading()) - brakeDist;
        return Math.max(-0.3, Math.min(1.0, JohnTuning.HEADING_KP * error)) / 2;
    }

    /** The tuner's brake: a tiny power against the motion (at least the heading power), motors floating. */
    private static void brake(Mecanum drivetrain, boolean strafe, int direction, double headingPower) {
        double brake = -BRAKING_POWER * direction;
        double minBrake = Math.abs(headingPower) + 0.001;
        brake = direction > 0 ? Math.min(brake, -minBrake) : Math.max(brake, minBrake);
        drivetrain.drive(strafe ? new DrivePowers(0, brake, headingPower) : new DrivePowers(brake, 0, headingPower), false);
    }

    private static double[] parse(String list) {
        String[] parts = list.split(",");
        double[] out = new double[parts.length];
        for (int i = 0; i < parts.length; i++) out[i] = Double.parseDouble(parts[i].trim());
        return out;
    }
}
