package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Barry.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Barry.Dashboard.Drawing;

/**
 * TeleOp OpMode demonstrating manual Mecanum drive with Pedro Pathing
 * while rendering the robot's real-time position on the FTC Dashboard Field Overlay.
 * Pressing START on gamepad1 commands Pedro Pathing to hold the target starting pose.
 */
@Config
@TeleOp(name = "Mecanum Drive", group = "BarryTest")
public class MecanumTeleOp extends LinearOpMode {

    // Configurable starting pose parameters (Heading in degrees)
    public static double START_X = 24.0;
    public static double START_Y = 24.0;
    public static double START_HEADING_DEG = 0.0;

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private boolean lastStart = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dual telemetry for Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize Pedro Pathing Follower from Constants
        follower = Constants.create(hardwareMap);
        follower.setPose(p.of(START_X, START_Y, START_HEADING_DEG));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read driver inputs from gamepad1 with deadband
            double forward = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0.0;
            double lateral = Math.abs(gamepad1.left_stick_x) > 0.05 ? -gamepad1.left_stick_x : 0.0;
            double heading = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0.0;

            boolean hasManualInput = (forward != 0.0 || lateral != 0.0 || heading != 0.0);

            // Trigger holding target goal pose when START button is pressed
            boolean currentStart = gamepad1.start;
            if (currentStart && !lastStart) {
                // Command Pedro follower to hold position at target pose
                follower.hold(p.of(START_X, START_Y, START_HEADING_DEG));
            }
            lastStart = currentStart;

            // Send manual control commands whenever there is manual input OR if currently in MANUAL mode
            if (hasManualInput || follower.mode() == Follower.Mode.MANUAL) {
                follower.manual(forward, lateral, heading);
            }

            follower.update();

            // Get current pose from Pedro localizer
            Pose currentPose = follower.pose();

            // Create a telemetry packet for FTC Dashboard with field overlay drawing
            TelemetryPacket packet = new TelemetryPacket(!Drawing.USE_CUSTOM_IMAGE);
            Drawing.drawRobot(packet.fieldOverlay(), currentPose);

            // Send telemetry packet to FTC Dashboard
            dashboard.sendTelemetryPacket(packet);

            // Display position data on Driver Station / Dashboard Telemetry
            telemetry.addData("X", currentPose.x());
            telemetry.addData("Y", currentPose.y());
            telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.heading()));
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.update();
        }
    }
}
