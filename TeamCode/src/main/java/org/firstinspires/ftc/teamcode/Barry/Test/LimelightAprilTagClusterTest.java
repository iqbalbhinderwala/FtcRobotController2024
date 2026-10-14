package org.firstinspires.ftc.teamcode.Barry.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Barry.Vision.ClusterDetection;
import org.firstinspires.ftc.teamcode.Barry.Vision.LimelightAprilTagClusterDetector;

import java.util.List;

/**
 * Limelight AprilTag Cluster Test
 *
 * Shows the center of each BIOBUZZ AprilTag cluster in view (see LimelightAprilTagClusterDetector).
 * Center is in Limelight camera space, meters: X right, Y down, Z forward from the lens.
 * "camera in cluster" is the camera pose in the cluster's frame (origin at the cluster center, X right
 * along the sticker, Y down, Z into the sticker, so a camera in front has negative Z).
 *
 * INSTRUCTIONS:
 * 1. Ensure Limelight is configured as "limelight" in the Robot Config.
 * 2. Ensure the Limelight has an AprilTag pipeline on Index 0 with tag size 82.55 mm (3.25 in).
 * 3. Point the camera at a cluster (tags 30-45). Try covering some tags: the center should
 *    stay about the same with 2, 3 or 4 tags visible.
 */
@TeleOp(name = "Limelight AprilTag Cluster Test", group = "Teleop")
public class LimelightAprilTagClusterTest extends LinearOpMode {

    private static final double METERS_TO_INCHES = 39.3701;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup dual telemetry for Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightAprilTagClusterDetector detector = new LimelightAprilTagClusterDetector();

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            List<ClusterDetection> clusters = detector.detect(result);

            telemetry.addData("Clusters in view", clusters.size());

            for (ClusterDetection c : clusters) {
                telemetry.addData(c.name,
                        "tags:%d | center(m) X:%+5.2f Y:%+5.2f Z:%+5.2f | range:%5.1f in | bearing:%+6.1f | elev:%+6.1f",
                        c.tagCount, c.x, c.y, c.z,
                        c.rangeMeters() * METERS_TO_INCHES, c.bearingDeg(), c.elevationDeg());

                ClusterDetection.CameraPose cam = c.cameraPose;
                if (cam == null) {
                    telemetry.addData(" camera in cluster", "needs 2+ tags");
                } else {
                    telemetry.addData(" camera in cluster",
                            "pos(m) X:%+5.2f Y:%+5.2f Z:%+5.2f | rot(deg) RX:%+6.1f RY:%+6.1f RZ:%+6.1f",
                            cam.x, cam.y, cam.z, cam.rxDeg, cam.ryDeg, cam.rzDeg);
                }
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
