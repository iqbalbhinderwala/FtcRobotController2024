package org.firstinspires.ftc.teamcode.John.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;

/**
 * Chooses John's autonomous setup before the match: alliance, start tile and start delay (see
 * AutoSetup for the buttons). Every change is saved to the blackboard right away, so just stop this
 * OpMode when done, then select [John] Auto, which only reads the setup and refuses to run without
 * it. Works in INIT and after START alike. The robot does not move: no drive hardware is used.
 *
 * The setup lasts until the Robot Controller app restarts; after a restart, run this again.
 */
@TeleOp(name = "[John] Game Setup", group = "John")
public class GameSetup extends LinearOpMode {

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        AutoSetup setup = new AutoSetup(blackboard);

        while (opModeInInit() || opModeIsActive()) {
            setup.update(gamepad1);
            setup.save(blackboard);

            telemetry.addLine("John Game Setup (saved on every change)");
            setup.addTelemetry(telemetry, true);
            telemetry.addLine("When done: stop this OpMode, then select [John] Auto.");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), setup.startPose());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
