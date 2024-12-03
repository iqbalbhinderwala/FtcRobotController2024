package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Log;

public class SamMainAuto_Impl {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.
    private Alliance.Side startSide;
    private boolean pushPiecesToMatchingZone;

    /* Declare Component members. */
    public SamIMUOmniDriveTrain nav = null;
    public SamJoints joints = null;
    public SamClaw claw = null;

    private ElapsedTime runtime = new ElapsedTime();

    public SamMainAuto_Impl(LinearOpMode opMode, Alliance.Side startSide, boolean pushPiecesToMatchingZone) {
        this.opMode = opMode;
        this.startSide = startSide;
        this.pushPiecesToMatchingZone = pushPiecesToMatchingZone;
    }

    // Initialize all hardware components.
    public void runOpMode() {
        nav = new SamIMUOmniDriveTrain(opMode);
        joints = new SamJoints(opMode);
        claw = new SamClaw(opMode);

        // Initialize components.
        nav.init();
        joints.init();
        claw.init();

        // Send telemetry message to signify robot waiting
        opMode.telemetry.addData("Status", "Ready to start.");

        joints.tryResetEncoders();
        if (joints.isFullyCalibrated()) {
            opMode.telemetry.addData("", "*** FULLY CALIBRATED ***");
        } else {
            opMode.telemetry.addData("", "WARNING: !!! NOT CALIBRATED !!!");
        }
        opMode.telemetry.update();

        // Wait for driver to press PLAY before robot allowed to move.
        opMode.waitForStart();
        runtime.reset();

        // Start components.
        joints.start();
        nav.resetOdometers(); // reset odometer encoders

        // ------ Start performing tasks ------

        // High clip bar is in front: 22in
        //   Split the drive in 2 segments:
        //      Close the claw so we don't drop the pre-loaded specimen
        //      Activate HIGHBAR joints preset
        //      Drive forward fast a safe distance while preset is active
        //      Wait for the HIGHBAR preset to complete without hitting the bar
        //      Drive at much slower power to perform the clipping

        // Clipping checkpoints (assume 47 inch to bar):
        final double X_START        =  0;
        final double X_CLIPBAR      = 22;
        final double X_PRESET_SAFE  = X_CLIPBAR - 8;
        final double X_CLIPPED      = X_CLIPBAR + 4;
        final double X_PARKED       = 10;

        final double Y_START        = 0; // -1.5 inches from middle of central seam
        final double Y_CORRIDOR     = -26.5; // observation corridor (negative towards right)

        final int BRAKING_TIME = 200; // Idle time between drive tasks (mSec)

        // Close the claw so we don't drop the pre-loaded specimen
        if (opMode.opModeIsActive()) {
            claw.closed();
        }

        // If starting on left side, move slightly right to clear the left bar
        if (startSide == Alliance.Side.LEFT) {
            nav.driveDistance(0, -6, 1);
            opMode.sleep(BRAKING_TIME);
        }
        Log.d("SAM::AUTO", "START "+(nav.getCurrentInchesOdometerX()));

        // Initiate HIGHBAR joints preset (non-blocking)
        if (opMode.opModeIsActive()) {
            joints.activatePreset(SamJoints.Pose.HIGHBAR);
        }

        // Drive a little while preset is active (blocking)
        if (opMode.opModeIsActive()) {
            nav.driveDistance(X_PRESET_SAFE, 0, 1);
            Log.d("SAM::AUTO", "PRESET "+(nav.getCurrentInchesOdometerX()));
        }

        // Wait for the HIGHBAR preset to complete
        while(opMode.opModeIsActive() && joints.isPresetActive()) {
            joints.stepActivePreset();
        }

        // Perform clipping by driving forward slowly to CLIPPING position.
        if (opMode.opModeIsActive()) {
            nav.driveDistance(X_CLIPPED-nav.getCurrentInchesOdometerX(), 0, 0.3);
            Log.d("SAM::AUTO", "CLIPPED"+(nav.getCurrentInchesOdometerX()));
        }

        // Open the claw
        if (opMode.opModeIsActive()) {
            claw.open();
            opMode.sleep(500);
        }

        // Drive back to 10 inches from rail at high power (blocking)
        if (opMode.opModeIsActive()) {
            nav.driveDistance(X_PARKED-nav.getCurrentInchesOdometerX(), 0, .7);
            Log.d("SAM::AUTO", "PARKED"+(nav.getCurrentInchesOdometerX()));
        }

        // Initiate PARKED joints preset (non-blocking)
        if (opMode.opModeIsActive()) {
            joints.activatePreset(SamJoints.Pose.PARKED);
        }

        // Push floor pieces into observation zone OR park
        if (pushPiecesToMatchingZone) {
            double mirrorY_if_LEFT_SIDE = (startSide == Alliance.Side.LEFT) ? -1 : 1;

            // Strafe to the matching corridor to get behind the pieces on the floor
            if (opMode.opModeIsActive()) {
                nav.driveDistance(0, Y_CORRIDOR*mirrorY_if_LEFT_SIDE - nav.getCurrentInchesOdometerY(), 1);
                opMode.sleep(BRAKING_TIME);
            }

            for (int i = 0; i < 3; i++) {
                // Drive back to 6 inches from rail at high power (blocking)
                if (opMode.opModeIsActive()) {
                    nav.driveDistance(51 - nav.getCurrentInchesOdometerX(), 0, 1);
                    opMode.sleep(BRAKING_TIME);
                }

                // Strafe right to parking zone (blocking)
                if (opMode.opModeIsActive()) {
                    nav.driveDistance(0, -9*mirrorY_if_LEFT_SIDE, 1);
                    opMode.sleep(BRAKING_TIME);
                }

                // Drive back to 10 inches from rail at high power (blocking)
                if (opMode.opModeIsActive()) {
                    nav.driveDistance(X_PARKED - nav.getCurrentInchesOdometerX(), 0, 1);
                    opMode.sleep(BRAKING_TIME);
                }
            } // for

            // Strafe away from wall (blocking)
            if (opMode.opModeIsActive()) {
                nav.driveDistance(0, 6*mirrorY_if_LEFT_SIDE, 1);
                opMode.sleep(BRAKING_TIME);
            }

        } else {
            // If not pushing pieces, then just park into observation zone on the right

            // Strafe right 2 tiles to parking zone (blocking)
            if (opMode.opModeIsActive()) {
                nav.driveDistance(0, -50, 1);
            }
        }

        // Wait for PARKED preset to complete
         while(opMode.opModeIsActive() && joints.isPresetActive()) {
             joints.stepActivePreset();
         }
    }
}
