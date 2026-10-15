package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.Barry.Field.HiveTagGeometry;

/**
 * Draws the nominal BIOBUZZ HIVE tag locations on the FTC Dashboard field (top-down) and optionally
 * swings the HIVES back and forth. Coordinates follow Drawing.CONVENTION (PEDRO or FTC), which is
 * switchable live from Dashboard; telemetry positions use the same convention.
 *
 * Positions come from HiveTagGeometry (Competition Manual dimensions, +/- 1 in). The top-down view
 * only shows the horizontal motion and the foreshortening of the tags; the heights (Z) change a lot
 * more and are shown in telemetry. The CELL that is up is drawn solid, the one that is down is faded.
 * All values below are live-editable from FTC Dashboard.
 */
@Config
@TeleOp(name = "Hive Tag Display", group = "BarryTest")
public class HiveTagDisplayTest extends LinearOpMode {

    public static boolean ANIMATE = true;
    /** The animation swings between -amplitude and +amplitude (the stable states are +/-30). */
    public static double TILT_AMPLITUDE_DEG = 30.0;
    /** Seconds the HIVE rests at each end (+/-amplitude) before swinging back. */
    public static double DWELL_SECONDS = 1.0;
    /** Seconds one swing from one end to the other takes. */
    public static double SWING_SECONDS = 1.0;

    /** Used when ANIMATE is off. Red +30 = audience CELL up, blue +30 = far CELL up (match start). */
    public static double RED_TILT_DEG = 30.0;
    public static double BLUE_TILT_DEG = 30.0;

    public static boolean SHOW_TAG_IDS = true;
    public static double DOWN_CELL_ALPHA = 0.25;

    private static final double MIN_LOOP_MS = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dual telemetry for Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            double redTilt;
            double blueTilt;
            if (ANIMATE) {
                // Same tilt for both HIVES: HiveTagGeometry mirrors blue, so the field stays symmetric
                redTilt = tiltAt(timer.seconds());
                blueTilt = redTilt;
            } else {
                redTilt = RED_TILT_DEG;
                blueTilt = BLUE_TILT_DEG;
            }

            // Field background, axes and grid (no robot), then the tags on top
            TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
            Canvas field = packet.fieldOverlay();
            Drawing.drawRobot(field, null);

            for (HiveTagGeometry.TagPose tag : HiveTagGeometry.allTagPoses(redTilt, blueTilt)) {
                drawTag(field, tag, tag.red ? redTilt : blueTilt);
            }
            field.setAlpha(1.0);

            telemetry.addData("Coordinates", Drawing.CONVENTION);
            addHiveTelemetry("Red", true, redTilt);
            addHiveTelemetry("Blue", false, blueTilt);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep((long) MIN_LOOP_MS);
        }
    }

    /**
     * Tilt at time t: rests at +amplitude for DWELL_SECONDS, eases down to -amplitude over
     * SWING_SECONDS, rests at -amplitude for DWELL_SECONDS, then eases back up.
     */
    private static double tiltAt(double t) {
        double dwell = Math.max(0.0, DWELL_SECONDS);
        double swing = Math.max(1e-3, SWING_SECONDS);
        double cycle = 2.0 * (dwell + swing);
        double u = ((t % cycle) + cycle) % cycle;

        if (u < dwell) {
            return TILT_AMPLITUDE_DEG;
        }
        u -= dwell;
        if (u < swing) {
            return TILT_AMPLITUDE_DEG * Math.cos(Math.PI * u / swing);
        }
        u -= swing;
        if (u < dwell) {
            return -TILT_AMPLITUDE_DEG;
        }
        u -= dwell;
        return -TILT_AMPLITUDE_DEG * Math.cos(Math.PI * u / swing);
    }

    private static void drawTag(Canvas field, HiveTagGeometry.TagPose tag, double tiltDeg) {
        // Field frame (= Drawing's FTC convention) -> the active drawing convention
        double[] center = toConvention(tag.x, tag.y, true);
        double x = center[0];
        double y = center[1];
        // Tag X axis is horizontal; rotate its direction the same way
        double[] dir = toConvention(tag.xAxis[0], tag.xAxis[1], false);
        double dirX = dir[0];
        double dirY = dir[1];
        // Top-down, the tag is shortened by the cosine of its tilt
        double height = HiveTagGeometry.TAG_SIZE * Math.abs(Math.cos(Math.toRadians(tiltDeg)));

        boolean up = tag.z > HiveTagGeometry.PIVOT_Z;
        field.setAlpha(up ? 1.0 : DOWN_CELL_ALPHA);
        Drawing.drawTag(field, x, y, dirX, dirY, HiveTagGeometry.TAG_SIZE, height,
                tag.red ? "red" : "blue", SHOW_TAG_IDS ? String.valueOf(tag.id) : null);
    }

    private void addHiveTelemetry(String name, boolean red, double tiltDeg) {
        HiveTagGeometry.TagPose audience = HiveTagGeometry.clusterPose(red, true, tiltDeg);
        HiveTagGeometry.TagPose far = HiveTagGeometry.clusterPose(red, false, tiltDeg);
        telemetry.addData(name + " HIVE tilt", "%+.1f deg", tiltDeg);
        double[] a = toConvention(audience.x, audience.y, true);
        double[] f = toConvention(far.x, far.y, true);
        // Keys must be unique per HIVE, or blue's lines replace red's
        telemetry.addData(name + " audience cluster", "x:%.1f y:%.1f z:%.1f in", a[0], a[1], audience.z);
        telemetry.addData(name + " far cluster", "x:%.1f y:%.1f z:%.1f in", f[0], f[1], far.z);
    }

    /**
     * Converts a field-frame (FTC) point or direction to the active Drawing.CONVENTION.
     * FTC is the field frame as is; Pedro is rotated 90 degrees with its origin at a corner.
     */
    private static double[] toConvention(double fieldX, double fieldY, boolean isPoint) {
        if (Drawing.CONVENTION == Drawing.CoordinateConvention.FTC) {
            return new double[]{fieldX, fieldY};
        }
        if (isPoint) {
            return new double[]{HiveTagGeometry.toPedroX(fieldY), HiveTagGeometry.toPedroY(fieldX)};
        }
        return new double[]{fieldY, -fieldX};
    }
}
