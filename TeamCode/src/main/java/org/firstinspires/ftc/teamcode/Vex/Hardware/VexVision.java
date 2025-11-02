package org.firstinspires.ftc.teamcode.Vex.Hardware;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VexVision {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.
    private Telemetry telemetry = null;
    /* Declare Component members. */
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Add resolution enumeration
    public enum Resolution {
        R_640x480(new Size(640, 480)),
        R_800x600(new Size(800, 600)),
        R_800x448(new Size(800, 448)),
        R_848x480(new Size(848, 480)),
        R_960x540(new Size(960, 540)),
        R_1280x720(new Size(1280, 720));

        private final Size size;

        Resolution(Size size) {
            this.size = size;
        }

        public Size getSize() {
            return size;
        }
    }

    public VexVision(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    // Initialize all hardware components.
    public void init() {
        // Initialize the AprilTag camera processor first
        initAprilTagProcessor(DEFAULT_RESOLUTION.getSize(), DEFAULT_COMPRESSION);

        // After camera was initialized, set low exposure time to reduce motion blur
        setManualExposure(DEFAULT_EXPOSURE_MS, DEFAULT_GAIN);
    }

    // Start all hardware components.
    public void start() {
    }

    public void stop() {
        // Save more CPU resources when camera is no longer needed.
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTagProcessor(Size resolution, boolean compressed) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(resolution);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        if (compressed) {
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // compressed, higher FPS
        } else {
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        }

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }   // end method initAprilTag()

    /**
     * Sets whether the AprilTag processor is enabled.
     * @param enabled true to enable the processor, false to disable it.
     */
    public void setAprilTagProcessorEnabled(boolean enabled) {
        if (visionPortal != null) {
            // Disable or re-enable the aprilTag processor at any time.
            visionPortal.setProcessorEnabled(aprilTag, enabled);
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }

    /**
     * Get the closest AprilTag "BlueTarget" or "RedTarget" detection from a list of current detections.
     *
     * @return The closest detection, or null if the list is empty or contains no valid metadata.
     */
    public AprilTagDetection getClosestTarget() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) {
            return null;
        }

        AprilTagDetection closest = null;
        double minRange = Double.MAX_VALUE;

        for (AprilTagDetection detection : currentDetections) {
            // Ensure the detection has metadata and a valid pose range
            if (detection.metadata != null && detection.metadata.name.contains("Target")
                    && detection.ftcPose.range < minRange) {

                minRange = detection.ftcPose.range;
                closest = detection;
            }
        }

        return closest;
    }

    @SuppressLint("DefaultLocale")
    public void addTelemetry() {
        AprilTagDetection detection = getClosestTarget();
        if (detection == null) {
            telemetry.addLine("No AprilTag detections.");
            return;
        }

        // DEBUG
        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

        telemetry.addLine(String.format("  Tag Field Pos: %s", detection.metadata.fieldPosition.toString()));

        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                detection.robotPose.getPosition().x,
                detection.robotPose.getPosition().y,
                detection.robotPose.getPosition().z));
        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    // From sample: ConceptAprilTagLocalization
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *            Z (up)
     *             ↑    Y(forward)
     *             |  /
     *             | /
     *             •────→ X (right)
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

    // Position and orientation of the camera on the robot
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private final Resolution DEFAULT_RESOLUTION = Resolution.R_960x540;
    private final boolean DEFAULT_COMPRESSION = true; // Use compression with high resolution
    private final int DEFAULT_EXPOSURE_MS = 6; // Use low exposure time to reduce motion blur
    private final int DEFAULT_GAIN = 250;
}
