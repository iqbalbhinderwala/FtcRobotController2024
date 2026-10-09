package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Vex Limelight AprilTag Test
 *
 * This OpMode demonstrates AprilTag detection using the Limelight 3A.
 * It displays the ID, pose, and distance for every tag in view.
 *
 * INSTRUCTIONS:
 * 1. Ensure Limelight is configured as "limelight" in the Robot Config.
 * 2. Ensure your Limelight has an AprilTag pipeline active on Index 0.
 * 3. Use the "Camera Stream" option on the Driver Station (3-dots menu) to see the actual video.
 */
@TeleOp(name = "Limelight AprilTag Test", group = "Teleop")
public class LimelightAprilTagTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dual telemetry for Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set telemetry to high frequency for "live" feel
        telemetry.setMsTransmissionInterval(11);

        // Switch to pipeline 0 (Ensure this is an AprilTag pipeline in the LL dashboard)
        limelight.pipelineSwitch(0);

        // Start polling for data
        limelight.start();

        int pipelineIndex = 0;
        int lastPipelineIndex = -1;

        while (!isStarted() && !isStopRequested()) {
            // Allow switching pipelines with D-pad during Init
            if (gamepad1.dpad_up) {
                pipelineIndex = Math.min(pipelineIndex + 1, 9);
                sleep(200);
            } else if (gamepad1.dpad_down) {
                pipelineIndex = Math.max(pipelineIndex - 1, 0);
                sleep(200);
            }

            // Only switch if the index has changed
            if (pipelineIndex != lastPipelineIndex) {
                limelight.pipelineSwitch(pipelineIndex);
                lastPipelineIndex = pipelineIndex;
            }

            LLStatus status = limelight.getStatus();
            telemetry.addLine("--- Limelight Status ---");
            telemetry.addData("Name", status.getName());
            telemetry.addData("Pipeline", "Index %d (%s)", status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("Temp", "%.1f C", status.getTemp());
            telemetry.addLine("-------------------------");
            telemetry.addLine("Is 'Camera Stream' in the 3-dot menu?");
            telemetry.update();
            sleep(100);
        }

        waitForStart();

        while (opModeIsActive()) {
            // Get the latest result from Limelight
            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addLine("!!! CRITICAL: result is NULL !!!");
            } else if (!result.isValid()) {
                telemetry.addLine("!!! result is INVALID !!!");
            } else {
                // 1. GET BOTPOSE (Robot's position on the field)
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    telemetry.addLine("--- Robot Field Pose ---");
                    telemetry.addData("Pos", "X:%.2f, Y:%.2f, Z:%.2f",
                            botpose.getPosition().x, botpose.getPosition().y, botpose.getPosition().z);
                    telemetry.addData("Rot", "Yaw:%.1f, Pitch:%.1f, Roll:%.1f",
                            botpose.getOrientation().getYaw(AngleUnit.DEGREES),
                            botpose.getOrientation().getPitch(AngleUnit.DEGREES),
                            botpose.getOrientation().getRoll(AngleUnit.DEGREES));
                }

                // 2. GET FIDUCIALS (AprilTag Specific Data)
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                telemetry.addLine("\n--- AprilTags in View (" + tags.size() + ") ---");

                for (LLResultTypes.FiducialResult tag : tags) {
                    telemetry.addLine("Tag ID: " + tag.getFiducialId());
                    telemetry.addData(" > Target (deg)", "X:%.2f, Y:%.2f", tag.getTargetXDegrees(), tag.getTargetYDegrees());

                    // Calculate distance to tag (if pose is available)
                    Pose3D tagPose = tag.getTargetPoseCameraSpace();
                    if (tagPose != null) {
                        double tx = tagPose.getPosition().x;
                        double ty = tagPose.getPosition().y;
                        double tz = tagPose.getPosition().z;

                        // Total Euclidean Distance
                        double distMeters = Math.sqrt(tx*tx + ty*ty + tz*tz);
                        double distInches = distMeters * 39.3701;

                        telemetry.addData(" > Rel Pose (m)", "X:%.2f, Y:%.2f, Z:%.2f", tx, ty, tz);
                        telemetry.addData(" > Distance", "%.2f m (%.1f in)", distMeters, distInches);
                    }
                }

                // 3. PERFORMANCE DATA
                telemetry.addLine("\n--- Performance ---");
                telemetry.addData("LL Latency", "%.1f ms", result.getCaptureLatency() + result.getTargetingLatency());
                telemetry.addData("FPS", "%d", (int)limelight.getStatus().getFps());

            }

            telemetry.update();
        }

        // Clean up
        limelight.stop();
    }
}