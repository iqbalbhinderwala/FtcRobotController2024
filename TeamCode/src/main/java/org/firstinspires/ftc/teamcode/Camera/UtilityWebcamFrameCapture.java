/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * This OpMode helps calibrate a webcam or RC phone camera, useful for AprilTag pose estimation
 * with the FTC VisionPortal.   It captures a camera frame (image) and stores it on the Robot Controller
 * (Control Hub or RC phone), with each press of the gamepad button X (or Square).
 * Full calibration instructions are here:
 *
 *  https://ftc-docs.firstinspires.org/camera-calibration
 *
 * In Android Studio, copy this class into your "teamcode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * In OnBot Java, use "Add File" to add this OpMode from the list of Samples.
 */

@TeleOp(name = "[Utility] Webcam Frame Capture", group = "Utility")
@Disabled
public class UtilityWebcamFrameCapture extends LinearOpMode
{
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
    final boolean USING_WEBCAM = true;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;

    @Override
    public void runOpMode()
    {
        List<Size> resolutions = new ArrayList<>();
        resolutions.add(new Size(640, 480));
        resolutions.add(new Size(800, 448));
        resolutions.add(new Size(800, 600));
        resolutions.add(new Size(848, 480));
        resolutions.add(new Size(960, 540));
        resolutions.add(new Size(1280, 720));

        int selectionIndex = 0;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Select Resolution");
            telemetry.addLine();

            for (int i = 0; i < resolutions.size(); i++) {
                String line = String.format(Locale.US, "%s %dx%d",
                        i == selectionIndex ? "[ -> ] " : "[ ] ",
                        resolutions.get(i).getWidth(), resolutions.get(i).getHeight());
                telemetry.addLine(line);
            }

            telemetry.addLine();
            telemetry.addLine("Use D-Pad Up/Down to select, A to confirm.");
            telemetry.update();

            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp) {
                selectionIndex = (selectionIndex - 1 + resolutions.size()) % resolutions.size();
            }
            lastDpadUp = dpadUp;

            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !lastDpadDown) {
                selectionIndex = (selectionIndex + 1) % resolutions.size();
            }
            lastDpadDown = dpadDown;

            if (gamepad1.a) {
                break;
            }
        }


        Size selectedResolution = resolutions.get(selectionIndex);

        VisionPortal portal;

        if (USING_WEBCAM)
        {
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(selectedResolution)
                    .build();
        }
        else
        {
            portal = new VisionPortal.Builder()
                    .setCamera(INTERNAL_CAM_DIR)
                    .setCameraResolution(selectedResolution)
                    .build();
        }

        while (!isStopRequested())
        {
            boolean x = gamepad1.x;

            if (x && !lastX)
            {
                portal.saveNextFrameRaw(String.format(Locale.US, "%dx%d-Frame%04d",
                        selectedResolution.getWidth(), selectedResolution.getHeight(), frameCount++));
                capReqTime = System.currentTimeMillis();
            }

            lastX = x;

            telemetry.addLine("######## Camera Capture Utility ########");
            telemetry.addLine(String.format(Locale.US, " > Resolution: %dx%d", selectedResolution.getWidth(), selectedResolution.getHeight()));
            telemetry.addLine(" > Press X (or Square) to capture a frame");
            telemetry.addData(" > Camera Status", portal.getCameraState());
            telemetry.addData(" > Frames Captured", frameCount);

            if (capReqTime != 0)
            {
                telemetry.addLine("Captured Frame!");
            }

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000)
            {
                capReqTime = 0;
            }

            telemetry.update();
        }
    }
}
