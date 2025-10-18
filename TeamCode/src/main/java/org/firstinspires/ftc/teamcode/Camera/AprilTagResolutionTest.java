/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/* FROM SAMPLES: ConceptAprilTag.java
 *
 */

@TeleOp(name = "[Utility] AprilTag Res Switch", group = "Utility")
public class AprilTagResolutionTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean useMJPEG = false;

    @Override
    public void runOpMode() {

        List<Size> resolutions = new ArrayList<>();
        // 4:3
        resolutions.add(new Size(640, 480));    // YUV2:30Hz ; MJPEG:29Hz
        resolutions.add(new Size(800, 600));    // YUV2:24Hz ; MJPEG:25Hz
        // 16:9
        resolutions.add(new Size(800, 448));    // YUV2:30Hz ; MJPEG:30Hz
        resolutions.add(new Size(848, 480));    // YUV2:30Hz ; MJPEG:28Hz
        resolutions.add(new Size(960, 540));    // YUV2:15Hz ; MJPEG:23Hz
        resolutions.add(new Size(1280, 720));   // YUV2:10Hz ; MJPEG:16Hz
        int resolutionIndex = resolutions.size()-1; // Default to highest resolution

        initAprilTag(resolutions.get(resolutionIndex));

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            boolean lastLeftBumper = false;
            boolean lastRightBumper = false;
            boolean lastY = false;

            while (opModeIsActive()) {

                // Change resolution on the fly
                boolean leftBumper = gamepad1.left_bumper;
                if (leftBumper && !lastLeftBumper) {
                    resolutionIndex = (resolutionIndex - 1 + resolutions.size()) % resolutions.size();
                    visionPortal.close();
                    initAprilTag(resolutions.get(resolutionIndex));
                }
                lastLeftBumper = leftBumper;

                boolean rightBumper = gamepad1.right_bumper;
                if (rightBumper && !lastRightBumper) {
                    resolutionIndex = (resolutionIndex + 1) % resolutions.size();
                    visionPortal.close();
                    initAprilTag(resolutions.get(resolutionIndex));
                }
                lastRightBumper = rightBumper;

                // Toggle stream format on the fly
                boolean yButton = gamepad1.y;
                if (yButton && !lastY) {
                    useMJPEG = !useMJPEG;
                    visionPortal.close();
                    initAprilTag(resolutions.get(resolutionIndex));
                }
                lastY = yButton;

                telemetry.addData("Camera",
                        resolutions.get(resolutionIndex).toString() + " (" + (useMJPEG ? "MJPEG" : "YUY2") + ")");
                telemetry.addData("FPS", "%.1f", visionPortal.getFps());

                telemetryAprilTag();

                telemetry.addLine("\nUse L/R bumpers to change resolution.");
                telemetry.addLine("Use Y to toggle stream format.");

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        if (visionPortal != null) {
            visionPortal.close();
        }

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(Size resolution) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .build();

        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(resolution);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        if (useMJPEG) {
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        } else {
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        }

        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class
