package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Checks the Pinpoint odometry against the real world (2026-10-31: after crossing the field the
 * pose was 5-7" off, the same way every run). The tests, in order (A = next, B = skip this one):
 *
 *  1. STILL: STILL_S without moving, square against the wall. x, y and heading must not creep.
 *  2. OUT: pull John straight out about a foot from the wall by hand (it cannot spin against it),
 *     press A. Moving him by hand does not spoil the heading check: the IMU follows that too.
 *     SPIN +: SPIN_TURNS turns counter-clockwise in place at TURN_POWER, open loop (no odometry
 *     feedback, which would hide its own error). Wrong pod offsets make the reported point circle
 *     once per turn; a least-squares fit (circle + slow drift, since a mecanum spin wanders a little)
 *     gives the lever-arm error and the offsets that would cancel it.
 *  3. SQUARE: push John back square against the same wall as at the start, press A. The heading
 *     should read the start heading again; what it is off by after SPIN_TURNS turns is the IMU's
 *     heading scale error.
 *  4. OUT, SPIN - and SQUARE again, clockwise.
 *  5. PUSH FORWARD: drive motors float. Line the intake B end up on a tile seam along a wall, press A,
 *     push John by hand exactly PUSH_IN (4 tiles = 96") along the wall, press A. The forward reading
 *     should be PUSH_IN, the sideways one about 0.
 *  6. PUSH SIDEWAYS: the same, pushing John to his left (shooter side).
 *
 * Start John square against a wall, with tape marks at his corners. The pose starts at (0, 0, 0):
 * +X is the intake B end, +Y the shooter side, heading counter-clockwise.
 *
 * OVERRIDE_OFFSETS with X_POD_OFFSET_IN / Y_POD_OFFSET_IN (set them on the Dashboard before INIT)
 * tries offsets without changing Constants: a right fix makes the next spin's circle near 0.
 * Logged to /sdcard/FIRST/john_logs/r<run>_john_odometry_test_<time>.csv.
 */
@Config
@TeleOp(name = "[John] Odometry Test", group = "JohnTest")
public class OdometryTest extends LinearOpMode {

    public static double STILL_S = 20.0;
    public static double TURN_POWER = 0.35;
    public static int SPIN_TURNS = 5;
    public static double PUSH_IN = 96.0;
    /** Try these pod offsets (inches, Pinpoint convention) instead of Constants.localizerConfig. */
    public static boolean OVERRIDE_OFFSETS = false;
    public static double X_POD_OFFSET_IN = 0.0;
    public static double Y_POD_OFFSET_IN = 0.9375;

    private static final String LOG_HEADER = "t_s,phase,x,y,h_deg,h_unwrapped_deg,enc_x,enc_y,pinpoint_status,turn_power,event";

    private enum Phase { STILL, OUT_1, SPIN_CCW, SQUARE_1, OUT_2, SPIN_CW, SQUARE_2, FORWARD_READY, FORWARD_PUSH,
        SIDEWAYS_READY, SIDEWAYS_PUSH, DONE }

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private CsvLog log;
    private final ElapsedTime clock = new ElapsedTime();
    private Phase phase = Phase.STILL;
    private double turnPower;
    private double unwrapped;
    private double lastHeading;
    private final List<String> results = new ArrayList<>();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        log = new CsvLog("john_odometry_test", LOG_HEADER);
        follower = Constants.create(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        DcMotor[] motors = {
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get())};
        double cfgX = Constants.localizerConfig.xPodOffset.get();
        double cfgY = Constants.localizerConfig.yPodOffset.get();
        if (OVERRIDE_OFFSETS) {
            pinpoint.setOffsets(X_POD_OFFSET_IN, Y_POD_OFFSET_IN, DistanceUnit.INCH);
            cfgX = X_POD_OFFSET_IN;
            cfgY = Y_POD_OFFSET_IN;
        }
        String offsets = String.format(Locale.US, "pod offsets x %.3f, y %.3f in%s", cfgX, cfgY,
                OVERRIDE_OFFSETS ? " (OVERRIDE)" : " (Constants)");

        while (opModeInInit()) {
            telemetry.addLine("Put John square against a wall (any side) and tape his corners. He spins away from it later.");
            telemetry.addLine(String.format(Locale.US, "START: %.0f s still, then %d turns each way at power %.2f.",
                    STILL_S, SPIN_TURNS, TURN_POWER));
            telemetry.addData("Odometry", offsets);
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();
        }
        if (isStopRequested()) {
            log.close();
            return;
        }
        gamepad1.resetEdgeDetection();
        follower.setPose(new Pose(0, 0, 0));
        follower.update();
        clock.reset();
        lastHeading = follower.pose().heading();
        row("start: " + offsets);

        ElapsedTime phaseTimer = new ElapsedTime();
        Pose phaseStart = follower.pose();
        double phaseUnwrapped = 0;
        List<double[]> spin = new ArrayList<>();   // {t, x, y, heading}
        final double squareRef = 0;                // square against the start wall reads 0

        while (opModeIsActive() && phase != Phase.DONE) {
            boolean next = gamepad1.aWasPressed(), skip = gamepad1.bWasPressed();
            follower.update();
            Pose p = follower.pose();
            unwrapped += normalize(p.heading() - lastHeading);
            lastHeading = p.heading();
            String event = null;
            Phase was = phase;

            switch (phase) {
                case STILL:
                    if (phaseTimer.seconds() >= STILL_S || skip) {
                        event = skip ? "still: skipped" : result(String.format(Locale.US,
                                "STILL %.0f s: moved %.2f in (x %+.2f, y %+.2f), heading %+.2f deg",
                                phaseTimer.seconds(), Math.hypot(p.x() - phaseStart.x(), p.y() - phaseStart.y()),
                                p.x() - phaseStart.x(), p.y() - phaseStart.y(), Math.toDegrees(normalize(p.heading() - phaseStart.heading()))));
                        phase = Phase.OUT_1;
                    }
                    break;
                case OUT_1:
                case OUT_2:
                    if (next) {
                        phase = phase == Phase.OUT_1 ? Phase.SPIN_CCW : Phase.SPIN_CW;
                    } else if (skip) {
                        event = "spin: skipped";
                        phase = phase == Phase.OUT_1 ? Phase.OUT_2 : Phase.FORWARD_READY;
                    }
                    break;
                case SPIN_CCW:
                case SPIN_CW: {
                    double dir = phase == Phase.SPIN_CCW ? 1 : -1;
                    double turned = (unwrapped - phaseUnwrapped) * dir;
                    if (skip || turned >= SPIN_TURNS * 2 * Math.PI) {
                        turnPower = 0;
                        event = skip ? "spin: skipped" : result(fitSpin(spin, dir > 0 ? "SPIN +" : "SPIN -", cfgX, cfgY));
                        phase = phase == Phase.SPIN_CCW ? Phase.SQUARE_1 : Phase.SQUARE_2;
                    } else {
                        turnPower = dir * TURN_POWER;
                        spin.add(new double[]{clock.seconds(), p.x(), p.y(), p.heading()});
                    }
                    break;
                }
                case SQUARE_1:
                case SQUARE_2:
                    if (next) {
                        // John is square against the start wall again: the heading should read squareRef
                        // (the start heading, whole turns later); what it reads instead is the scale error
                        double err = Math.toDegrees(normalize(p.heading() - squareRef));
                        event = result(String.format(Locale.US,
                                "HEADING after %d turns %s: reads %+.2f deg off (%+.3f deg per turn)",
                                SPIN_TURNS, phase == Phase.SQUARE_1 ? "CCW" : "CW", err, err / SPIN_TURNS));
                        follower.setPose(new Pose(p.x(), p.y(), squareRef));   // start the next test from the truth
                        lastHeading = squareRef;
                        phase = phase == Phase.SQUARE_1 ? Phase.OUT_2 : Phase.FORWARD_READY;
                    } else if (skip) {
                        event = "square: skipped";
                        phase = phase == Phase.SQUARE_1 ? Phase.OUT_2 : Phase.FORWARD_READY;
                    }
                    break;
                case FORWARD_READY:
                case SIDEWAYS_READY:
                    if (next) {
                        event = phase == Phase.FORWARD_READY ? "push forward: start" : "push sideways: start";
                        phase = phase == Phase.FORWARD_READY ? Phase.FORWARD_PUSH : Phase.SIDEWAYS_PUSH;
                    } else if (skip) {
                        event = "push: skipped";
                        phase = phase == Phase.FORWARD_READY ? Phase.SIDEWAYS_READY : Phase.DONE;
                    }
                    break;
                case FORWARD_PUSH:
                case SIDEWAYS_PUSH:
                    if (next) {
                        // In John's frame at the push start: forward along his X, sideways along his Y
                        double dx = p.x() - phaseStart.x(), dy = p.y() - phaseStart.y();
                        double c = Math.cos(phaseStart.heading()), s = Math.sin(phaseStart.heading());
                        double fwd = dx * c + dy * s, side = -dx * s + dy * c;
                        boolean forward = phase == Phase.FORWARD_PUSH;
                        double along = forward ? fwd : side, across = forward ? side : fwd;
                        event = result(String.format(Locale.US,
                                "PUSH %s %.0f in: read %.2f in (%+.2f%%), %+.2f in across, heading %+.2f deg",
                                forward ? "FORWARD" : "SIDEWAYS", PUSH_IN, along, (Math.abs(along) / PUSH_IN - 1) * 100, across,
                                Math.toDegrees(normalize(p.heading() - phaseStart.heading()))));
                        phase = forward ? Phase.SIDEWAYS_READY : Phase.DONE;
                    } else if (skip) {
                        event = "push: skipped";
                        phase = phase == Phase.FORWARD_PUSH ? Phase.SIDEWAYS_READY : Phase.DONE;
                    }
                    break;
                default:
                    break;
            }

            if (phase != was) {
                phaseTimer.reset();
                phaseStart = follower.pose();
                phaseUnwrapped = unwrapped;
                spin.clear();
                boolean pushing = phase == Phase.FORWARD_READY || phase == Phase.FORWARD_PUSH
                        || phase == Phase.SIDEWAYS_READY || phase == Phase.SIDEWAYS_PUSH;
                for (DcMotor m : motors) {   // float to push by hand, brake otherwise
                    m.setZeroPowerBehavior(pushing ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            follower.manual(new DrivePowers(0, 0, turnPower));
            row(event);

            telemetry.addData("Test", phase + "   (A next, B skip)");
            telemetry.addLine(instructions(phase, phaseTimer.seconds()));
            telemetry.addData("Pose", "x %.2f  y %.2f  h %.2f deg  (turned %.0f deg)", p.x(), p.y(),
                    Math.toDegrees(p.heading()), Math.toDegrees(unwrapped));
            telemetry.addData("Odometry", offsets);
            for (String r : results) {
                telemetry.addLine(r);
            }
            telemetry.update();
        }
        follower.manual(DrivePowers.zero());
        follower.update();
        while (opModeIsActive()) {   // keep the results on screen
            telemetry.addData("Test", "DONE (results are also in the log)");
            for (String r : results) {
                telemetry.addLine(r);
            }
            telemetry.update();
            sleep(100);
        }
        log.close();
    }

    private String instructions(Phase phase, double s) {
        switch (phase) {
            case STILL: return String.format(Locale.US, "Hands off: %.0f s of standing still.", Math.max(0, STILL_S - s));
            case OUT_1: case OUT_2: return "Pull John straight out about a foot from the wall (about 4 ft clear around him), then press A to spin.";
            case SPIN_CCW: return "Spinning counter-clockwise. Stay clear.";
            case SPIN_CW: return "Spinning clockwise. Stay clear.";
            case SQUARE_1: case SQUARE_2: return "Push John back square against the SAME wall, facing the same way as at the start, then press A.";
            case FORWARD_READY: return String.format(Locale.US, "Motors float. Put John against a wall, intake B end on a tile seam, then press A (then push %.0f in).", PUSH_IN);
            case FORWARD_PUSH: return String.format(Locale.US, "Push John %.0f in toward his intake B end, along the wall, then press A.", PUSH_IN);
            case SIDEWAYS_READY: return String.format(Locale.US, "Put John with his right side on a tile seam, then press A (then push %.0f in to his left).", PUSH_IN);
            case SIDEWAYS_PUSH: return String.format(Locale.US, "Push John %.0f in to his left (shooter side), then press A.", PUSH_IN);
            default: return "";
        }
    }

    /**
     * Fits the spin samples as p = c + c' t + R(h) d: c drifts slowly (the spin wanders), d is the
     * reported point's lever arm from the real center of rotation in John's frame. Pod offset
     * errors make d: a forward-pod (X) offset error moves the point sideways, a strafe-pod (Y)
     * offset error moves it forward (see the comments below).
     */
    private String fitSpin(List<double[]> s, String name, double cfgX, double cfgY) {
        if (s.size() < 30) {
            return name + ": too few samples to fit";
        }
        double t0 = s.get(0)[0];
        double[][] a = new double[6][6];
        double[] b = new double[6];
        for (double[] q : s) {
            double t = q[0] - t0, c = Math.cos(q[3]), sn = Math.sin(q[3]);
            double[][] rows = {{1, 0, t, 0, c, -sn}, {0, 1, 0, t, sn, c}};
            double[] obs = {q[1], q[2]};
            for (int r = 0; r < 2; r++) {
                for (int i = 0; i < 6; i++) {
                    b[i] += rows[r][i] * obs[r];
                    for (int j = 0; j < 6; j++) {
                        a[i][j] += rows[r][i] * rows[r][j];
                    }
                }
            }
        }
        double[] u = solve(a, b);
        if (u == null) {
            return name + ": fit failed";
        }
        double dx = u[4], dy = u[5];
        double rms = 0;
        for (double[] q : s) {
            double t = q[0] - t0, c = Math.cos(q[3]), sn = Math.sin(q[3]);
            double ex = q[1] - (u[0] + u[2] * t + c * dx - sn * dy);
            double ey = q[2] - (u[1] + u[3] * t + sn * dx + c * dy);
            rms += ex * ex + ey * ey;
        }
        rms = Math.sqrt(rms / s.size());
        // The reported point sits d from the real center, in John's frame:
        //   d.x (forward) = true Y-pod offset - configured one; d.y (left) = true X-pod offset - configured one
        // (from the Pinpoint's offset convention). Verify a fix with OVERRIDE_OFFSETS: the next circle
        // should be near 0; if it doubles, the signs are the other way round.
        return String.format(Locale.US,
                "%s: circle %.2f in (lever arm fwd %+.2f, left %+.2f), drift %.2f in/s, fit rms %.2f in. "
                        + "Suggested offsets: x %.3f, y %.3f in",
                name, Math.hypot(dx, dy), dx, dy, Math.hypot(u[2], u[3]), rms, cfgX + dy, cfgY + dx);
    }

    /** Gaussian elimination with partial pivoting; null if singular. */
    private static double[] solve(double[][] a, double[] b) {
        int n = b.length;
        for (int col = 0; col < n; col++) {
            int piv = col;
            for (int r = col + 1; r < n; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[piv][col])) {
                    piv = r;
                }
            }
            if (Math.abs(a[piv][col]) < 1e-9) {
                return null;
            }
            double[] tr = a[col]; a[col] = a[piv]; a[piv] = tr;
            double tb = b[col]; b[col] = b[piv]; b[piv] = tb;
            for (int r = col + 1; r < n; r++) {
                double f = a[r][col] / a[col][col];
                b[r] -= f * b[col];
                for (int k = col; k < n; k++) {
                    a[r][k] -= f * a[col][k];
                }
            }
        }
        double[] x = new double[n];
        for (int r = n - 1; r >= 0; r--) {
            double sum = b[r];
            for (int k = r + 1; k < n; k++) {
                sum -= a[r][k] * x[k];
            }
            x[r] = sum / a[r][r];
        }
        return x;
    }

    private String result(String text) {
        results.add(text);
        return text;
    }

    private void row(String event) {
        Pose p = follower.pose();
        log.row(clock.seconds(), phase, p.x(), p.y(), Math.toDegrees(p.heading()), Math.toDegrees(unwrapped),
                pinpoint.getEncoderX(), pinpoint.getEncoderY(), pinpoint.getDeviceStatus(), turnPower, event);
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
