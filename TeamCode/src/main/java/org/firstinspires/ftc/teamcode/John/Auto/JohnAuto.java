package org.firstinspires.ftc.teamcode.John.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;

/**
 * John's autonomous. The alliance, start tile and start delay come from [John] Game Setup (run it
 * before the match; see AutoSettings). INIT only shows them and previews the start pose on the
 * Dashboard field; no buttons are used, so nothing can be changed while it waits. Without a saved
 * Game Setup it refuses to run: the robot does not move at all.
 *
 * START: the follower is told the start pose, waits the chosen delay while holding it, then runs the
 * routine. The routine is empty for now: it just holds the start pose. The pose is kept in the
 * blackboard for a following TeleOp (AutoSettings.loadPose).
 */
@Autonomous(name = "[John] Auto", group = "John", preselectTeleOp = "[John] Drive Test")
public class JohnAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Follower follower = Constants.create(hardwareMap);
        OdometryWatchdog watchdog = new OdometryWatchdog(hardwareMap, "auto");
        boolean haveSettings = AutoSettings.isSaved(blackboard);
        AutoSettings settings = new AutoSettings(blackboard);

        // INIT: show the Game Setup and the start pose (no buttons)
        while (opModeInInit()) {
            if (haveSettings) {
                telemetry.addLine("Game Setup:");
                settings.addTelemetry(telemetry, false);
            } else {
                telemetry.addLine("NO GAME SETUP: this autonomous will not run.");
                telemetry.addLine("Stop, run [John] Game Setup, then INIT [John] Auto again.");
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            if (haveSettings) {
                Drawing.drawRobot(packet.fieldOverlay(), settings.startPose());
            }
            dashboard.sendTelemetryPacket(packet);
        }
        if (isStopRequested()) {
            return;
        }
        if (!haveSettings) {
            // Refuse to run: no setPose, no hold, the motors stay off
            while (opModeIsActive()) {
                telemetry.addLine("NO GAME SETUP: autonomous not run. Run [John] Game Setup before the match.");
                telemetry.update();
            }
            return;
        }

        Pose start = settings.startPose();
        follower.setPose(start);
        watchdog.poseSet();
        follower.update();
        follower.hold(start);

        ElapsedTime clock = new ElapsedTime();
        boolean routineStarted = false;
        while (opModeIsActive()) {
            if (!routineStarted && clock.seconds() >= settings.delayS()) {
                routineStarted = true;
                // The routine goes here (schedule the first path, shoot, intake ...).
            }
            follower.update();
            OdometryWatchdog.State state = watchdog.check(follower);
            AutoSettings.savePose(blackboard, follower.pose());

            telemetry.addData("Alliance / tile", "%s / %s", settings.alliance(), settings.tile());
            telemetry.addData("Time", "%.1f s%s", clock.seconds(),
                    routineStarted ? "" : String.format("  (waiting %.0f s delay)", settings.delayS()));
            telemetry.addData("Pose (FTC)", Drawing.format(follower.pose()));
            telemetry.addData("Pinpoint", "%s, %s", state, watchdog.status());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), follower.pose());
            dashboard.sendTelemetryPacket(packet);
        }
        watchdog.close();
    }
}
