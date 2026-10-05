package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Barry.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;

/**
 * TeleOp OpMode demonstrating manual Mecanum drive with Pedro Pathing
 * while rendering the robot's real-time position on the FTC Dashboard Field Overlay.
 */
@TeleOp(name = "Mecanum Drive", group = "BarryTest")
public class MecanumTeleOp extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dual telemetry for Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize Pedro Pathing Follower from Constants
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read driver inputs from gamepad1
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double heading = -gamepad1.right_stick_x;

            // Apply manual driving powers to Pedro Follower
            follower.manual(forward, lateral, heading);
            follower.update();

            // Get current pose from Pedro localizer
            Pose currentPose = follower.pose();

            // Create a telemetry packet for FTC Dashboard with field overlay drawing
            TelemetryPacket packet = new TelemetryPacket();
            Drawing.drawRobot(packet.fieldOverlay(), currentPose);

            // Send telemetry packet to FTC Dashboard
            dashboard.sendTelemetryPacket(packet);

            // Display position data on Driver Station / Dashboard Telemetry
            telemetry.addData("X", currentPose.x());
            telemetry.addData("Y", currentPose.y());
            telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.heading()));
            telemetry.update();
        }
    }
}
