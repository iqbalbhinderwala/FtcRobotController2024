package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        // ------ Start performing tasks ------

        // High clip bar is in front: 23in +/- 4in
        //   Split the drive in 2 segments:
        //      Close the claw so we don't drop the pre-loaded specimen
        //      Activate HIGHBAR joints preset
        //      Drive forward 12 inches at high power
        //      Wait for the HIGHBAR preset to complete
        //      Drive 8 inches at much slower power to perform the clipping


        // Close the claw so we don't drop the pre-loaded specimen
        if (opMode.opModeIsActive()) {
            claw.closed();
        }

        // Initiate HIGHBAR joints preset (non-blocking)
        if (opMode.opModeIsActive()) {
            joints.activatePreset(SamJoints.Pose.HIGHBAR);
        }

        // Drive forward 12 inches at high power (blocking)
        //   (high clip bar is in front: 16in +/- 4in)
        if (opMode.opModeIsActive()) {
            nav.driveDistance(23 - 5, 0, 1);
        }

        // Wait for the HIGHBAR preset to complete
        while(opMode.opModeIsActive() && joints.isPresetActive()) {
            joints.stepActivePreset();
        }

        // Perform clipping by driving 8 inches slowly
        if (opMode.opModeIsActive()) {
            nav.driveDistance(5 + 3, 0, 0.2);
        }

        // Open the claw
        if (opMode.opModeIsActive()) {
            claw.open();
        }

        // Drive back 16 inches at high power (blocking)
        if (opMode.opModeIsActive()) {
            nav.driveDistance(-6, 0, 1);
        }

        // Initiate PARKED joints preset (non-blocking)
        if (opMode.opModeIsActive()) {
            joints.activatePreset(SamJoints.Pose.PARKED);
        }

        opMode.sleep(500);
        // Strafe right to parking zone (blocking)
        double DISTANCE_PARKING_TO_HIGHBAR = 32;
        if (opMode.opModeIsActive()) {
            nav.driveDistance(0, -DISTANCE_PARKING_TO_HIGHBAR, 1);
        }

        // Wait for PARKED preset to complete
        while(opMode.opModeIsActive() && joints.isPresetActive()) {
            joints.stepActivePreset();
        }

        // TUrn from 0 to 90 to 180 degree heading
        nav.turnToHeading(1, 90);
        nav.turnToHeading(1, 180);

        joints.activatePreset(SamJoints.Pose.RAIL);

        nav.driveDistance(-3, 0, .3);

        // Wait for PARKED preset to complete
        while(opMode.opModeIsActive() && joints.isPresetActive()) {
            joints.stepActivePreset();
        }

        nav.driveDistance(5, 0, .2);
        opMode.sleep(500);
        claw.closed();
        opMode.sleep(500);
        joints.activatePreset(SamJoints.Pose.RAIL_UP);
        opMode.sleep(100);
        nav.driveDistance(-5, 0, 0.2);

        // Turn from 180 to 90 to 0 heading
        nav.turnToHeading(1, 90);
        nav.turnToHeading(1, 0);

        joints.activatePreset(SamJoints.Pose.HIGHBAR);
        // Wait for HIGHBAR preset to complete
        while(opMode.opModeIsActive() && joints.isPresetActive()) {
            joints.stepActivePreset();
        }

        nav.driveDistance(-12, 0, 1);

        nav.driveDistance(0, DISTANCE_PARKING_TO_HIGHBAR, 1);

    }
}
