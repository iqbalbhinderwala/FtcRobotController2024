package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Log;

public class SamMainAuto_Impl {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.
    private Alliance.Side startSide;

    /* Declare Component members. */
    public SamIMUOmniDriveTrain nav = null;
    public SamJoints joints = null;
    public SamClaw claw = null;

    private ElapsedTime runtime = new ElapsedTime();

    public SamMainAuto_Impl(LinearOpMode opMode, Alliance.Side startSide) {
        this.opMode = opMode;
        this.startSide = startSide;
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

        // High clip bar is in front: 23in
        //   Split the drive in 2 segments:
        //      Close the claw so we don't drop the pre-loaded specimen
        //      Activate HIGHBAR joints preset
        //      Drive forward 23-8 inches at high power
        //      Wait for the HIGHBAR preset to complete without hitting the bar
        //      Drive 8 inches at much slower power to perform the clipping

        // Clipping checkpoints (assume 47 inch to bar):
        final double X_START        =  0;
        final double X_CLIPBAR      = 22;
        final double X_PRESET_SAFE  = X_CLIPBAR - 8;
        final double X_CLIPPED      = X_CLIPBAR + 4;
        final double X_PARKED       = 6; 

        final double Y_START        = 0;
        final double Y_PARKED       = -50; // park right (negative)


        // Close the claw so we don't drop the pre-loaded specimen
        if (opMode.opModeIsActive()) {
            claw.closed();
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
        }

        opMode.sleep(1000);

        // Drive back to 6 inches from rail at high power (blocking)
        if (opMode.opModeIsActive()) {
            nav.driveDistance(X_PARKED-nav.getCurrentInchesOdometerX(), 0, .7);
            Log.d("SAM::AUTO", "PARKED"+(nav.getCurrentInchesOdometerX()));
        }

         // Initiate PARKED joints preset (non-blocking)
         if (opMode.opModeIsActive()) {
             joints.activatePreset(SamJoints.Pose.PARKED);
         }

         // Strafe right to parking zone (blocking)
         if (opMode.opModeIsActive()) {
             nav.driveDistance(0, Y_PARKED, 1);
         }

         // Wait for PARKED preset to complete
         while(opMode.opModeIsActive() && joints.isPresetActive()) {
             joints.stepActivePreset();
         }

         // TUrn from 0 to 90 to 180 degree heading
//         nav.turnToHeading(1, 90);
//         nav.turnToHeading(1, 180);

        // joints.activatePreset(SamJoints.Pose.RAIL);

        // nav.driveDistance(-3, 0, .3);

        // // Wait for RAIL preset to complete
        // while(opMode.opModeIsActive() && joints.isPresetActive()) {
        //     joints.stepActivePreset();
        // }

        // nav.driveDistance(5, 0, .2);
        // opMode.sleep(500);
        // claw.closed();
        // opMode.sleep(500);
        // joints.activatePreset(SamJoints.Pose.RAIL_UP);
        // opMode.sleep(100);
        // nav.driveDistance(-5, 0, 0.2);

        // // Turn from 180 to 90 to 0 heading
        // nav.turnToHeading(1, 90);
        // nav.turnToHeading(1, 0);

        // joints.activatePreset(SamJoints.Pose.HIGHBAR);
        // // Wait for HIGHBAR preset to complete
        // while(opMode.opModeIsActive() && joints.isPresetActive()) {
        //     joints.stepActivePreset();
        // }

        // nav.driveDistance(-12, 0, 1);

        // nav.driveDistance(0, DISTANCE_PARKING_TO_HIGHBAR, 1);

    }
}
