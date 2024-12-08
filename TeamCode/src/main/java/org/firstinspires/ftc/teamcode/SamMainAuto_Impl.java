package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Log;

public class SamMainAuto_Impl {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.
    private Alliance.Side startSide;
    private boolean enableClip;
    private boolean pushPiecesToMatchingZone;

    /* Declare Component members. */
    public SamIMUOmniDriveTrain nav = null;
    public SamJoints joints = null;
    public SamClaw claw = null;

    private ElapsedTime runtime = new ElapsedTime();

    public SamMainAuto_Impl(LinearOpMode opMode, Alliance.Side startSide, boolean enableClip, boolean pushPiecesToMatchingZone) {
        this.opMode = opMode;
        this.startSide = startSide;
        this.enableClip = enableClip;
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
        nav.resetHeading(); // reset IMU heading (yaw)

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
        final double X_PARKED       = 9;
        final double X_PUSH         = 51;

        final double Y_START        = 0; // -1.5 inches from middle of central seam
        final double Y_CORRIDOR     = -26.5; // observation corridor (negative towards right)

        final int BRAKING_TIME = 200; // Idle time between drive tasks (mSec)

        LogCurrentState("START");

        if (enableClip) {

            // Close the claw so we don't drop the pre-loaded specimen
            if (opMode.opModeIsActive()) {
                claw.closed();
            }

            // If starting on left side, move slightly right to clear the left bar
            if (startSide == Alliance.Side.LEFT) {
                nav.driveDistance(0, -6, 1);
                opMode.sleep(BRAKING_TIME);
                LogCurrentState("Strafe by dY=-6");
            }

            // Initiate HIGHBAR joints preset (non-blocking)
            if (opMode.opModeIsActive()) {
                joints.activatePreset(SamJoints.Pose.HIGHBAR);
            }

            // Drive a little while preset is active (blocking)
            if (opMode.opModeIsActive()) {
                nav.driveDistance(X_PRESET_SAFE, 0, 1);
                LogCurrentState("Go to X_PRESET_SAFE=" + X_PRESET_SAFE);
            }

            // Wait for the HIGHBAR preset to complete
            while (opMode.opModeIsActive() && joints.isPresetActive()) {
                joints.stepActivePreset();
            }

            // Perform clipping by driving forward slowly to CLIPPING position.
            if (opMode.opModeIsActive()) {
                nav.driveDistance(X_CLIPPED - nav.getCurrentInchesOdometerX(), 0, 0.3);
                LogCurrentState("Go to X_CLIPPED=" + X_CLIPPED);
            }
        }

        // Open the claw
        if (opMode.opModeIsActive()) {
            claw.open();
            opMode.sleep(500);
        }

        // During clipping, the robot may have rotated. Ensure still heading zero deg.
        if (opMode.opModeIsActive()) {
            nav.turnToHeading(1, 0);
            LogCurrentState("Heading=0");
        }

        // Drive back to 10 inches from rail at high power (blocking)
        if (opMode.opModeIsActive()) {
            nav.driveDistance(X_PARKED-nav.getCurrentInchesOdometerX(), 0, .7);
            LogCurrentState("Go to X_PARKED="+X_PARKED);
        }

        // Initiate PARKED joints preset (non-blocking)
        if (opMode.opModeIsActive()) {
            joints.activatePreset(SamJoints.Pose.PARKED);
        }

        // Zero heading
        if (opMode.opModeIsActive()) {
            nav.turnToHeading(1, 0);
            LogCurrentState("Heading=0");
        }

        // Push floor pieces into observation zone OR park
        if (pushPiecesToMatchingZone) {
            double mirrorY_if_LEFT_SIDE = (startSide == Alliance.Side.LEFT) ? -1 : 1;

            // Strafe to the matching corridor to get behind the pieces on the floor
            if (opMode.opModeIsActive()) {
                nav.driveDistance(0, Y_CORRIDOR*mirrorY_if_LEFT_SIDE - nav.getCurrentInchesOdometerY(), 1);
                opMode.sleep(BRAKING_TIME);
                LogCurrentState("Go to Y_CORRIDOR="+(Y_CORRIDOR*mirrorY_if_LEFT_SIDE));
            }

            // Zero heading
            if (opMode.opModeIsActive()) {
                nav.turnToHeading(1, 0);
                LogCurrentState("Heading=0");
            }

            for (int i = 0; i < 3; i++) {
                // Drive forward to PUSH PIECES at high power (blocking)
                if (opMode.opModeIsActive()) {
                    nav.driveDistance(X_PUSH - nav.getCurrentInchesOdometerX(), 0, 1);
                    opMode.sleep(BRAKING_TIME);
                    LogCurrentState("Go to X_PUSH="+X_PUSH);
                }

                // Strafe right to parking zone (blocking)
                if (opMode.opModeIsActive()) {
                    nav.driveDistance(0, -9*mirrorY_if_LEFT_SIDE, 1);
                    opMode.sleep(BRAKING_TIME);
                    LogCurrentState("Move by dY="+(-9*mirrorY_if_LEFT_SIDE));
                }

                if (startSide == Alliance.Side.LEFT && i == 0) {
                    if (opMode.opModeIsActive()) {
                        nav.turnToHeading(1, -10);
                        LogCurrentState("Heading=-10");
                    }
                }

                if (startSide == Alliance.Side.LEFT && i == 1) {
                    if (opMode.opModeIsActive()) {
                        nav.turnToHeading(1, 0);
                        LogCurrentState("Heading=0");
                    }
                }

                // Drive back to 10 inches from rail at high power (blocking)
                if (opMode.opModeIsActive()) {
                    nav.driveDistance(X_PARKED - nav.getCurrentInchesOdometerX(), 0, 1);
                    opMode.sleep(BRAKING_TIME);
                    LogCurrentState("Go to X_PARKED="+X_PARKED);
                }
            } // for

            // Strafe away from wall (blocking)
            if (opMode.opModeIsActive()) {
                nav.driveDistance(0, 6*mirrorY_if_LEFT_SIDE, 1);
                opMode.sleep(BRAKING_TIME);
                LogCurrentState("Move by dY="+(6*mirrorY_if_LEFT_SIDE));
            }

        } else {
            // If not pushing pieces, then just park into observation zone on the right

            // Strafe right 2 tiles to parking zone (blocking)
            if (opMode.opModeIsActive()) {
                nav.driveDistance(0, -50, 1);
                opMode.sleep(BRAKING_TIME);
                LogCurrentState("Move by dY=-50");
            }
        }

        // Wait for PARKED preset to complete
         while(opMode.opModeIsActive() && joints.isPresetActive()) {
             joints.stepActivePreset();
         }
    }

    private void LogCurrentState(String phase) {
        Log.d("SAM::AUTO", "X_in: "+nav.getCurrentInchesOdometerX()
                + " Y_in: "+nav.getCurrentInchesOdometerY() + " H_deg: "+nav.getHeading() + " - " + phase);
    }
}
