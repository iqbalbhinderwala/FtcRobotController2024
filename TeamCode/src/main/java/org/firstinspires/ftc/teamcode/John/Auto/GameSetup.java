package org.firstinspires.ftc.teamcode.John.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;

/**
 * Chooses John's autonomous settings before the match: alliance, start tile and start delay (see
 * AutoSettings for the buttons). Every change is saved to the blackboard right away, so just stop
 * this OpMode when done, then select [John] Auto, which only reads the settings and refuses to run
 * without them. The robot does not move: no drive hardware is used.
 *
 * Listed with the autonomous OpModes, next to [John] Auto. Use it in INIT; START is not needed (the
 * 30 s autonomous timer would just stop it, with the settings already saved).
 *
 * The settings last until the Robot Controller app restarts; after a restart, run this again.
 */
@Autonomous(name = "[John] Game Setup", group = "John")
public class GameSetup extends LinearOpMode {

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        AutoSettings settings = new AutoSettings(blackboard);

        while (opModeInInit() || opModeIsActive()) {
            settings.update(gamepad1);
            settings.save(blackboard);

            telemetry.addLine("John Game Setup (saved on every change)");
            settings.addTelemetry(telemetry, true);
            telemetry.addLine("When done: stop this OpMode, then select [John] Auto.");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), settings.startPose());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
