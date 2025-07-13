package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="[Sam] ***** MAIN TELEOP *****", group="Sam")
@Disabled
public class SamMainTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare Component members. */
    public SamIMUOmniDriveTrain nav = null;
    public SamJoints joints = null;
    public SamClaw claw = null;

    double MAX_DRIVE_SPEED  = 1;
    double MAX_STRAFE_SPEED = 1;
    double MAX_TURN_SPEED   = 1;

    ElapsedTime lastPress = new ElapsedTime();
    ElapsedTime lastLeftBumper = new ElapsedTime();
    ElapsedTime lastRightBumper = new ElapsedTime();
    final double BUTTON_DELAY = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        nav = new SamIMUOmniDriveTrain(this);
        joints = new SamJoints(this);
        claw = new SamClaw(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Initialize components.
        nav.init();
        joints.init();
        claw.init();

        // Start components.
        joints.start();
        claw.closed();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // GAMEPAD 2 - Navigation
            double drive  = -gamepad2.left_stick_y  * MAX_DRIVE_SPEED;
            double strafe = -gamepad2.left_stick_x  * MAX_STRAFE_SPEED;
            double turn   = -gamepad2.right_stick_x * MAX_TURN_SPEED;
            nav.moveRobot(drive, strafe, turn);

            // GAMEPAD 1 - ARM CONTROL

            // Claw
            if (gamepad2.left_bumper && lastLeftBumper.seconds() > BUTTON_DELAY) {
                lastLeftBumper.reset();
                claw.toggle();
            }

            // Attempt joints calibration
            if (!joints.isFullyCalibrated())
            {
                joints.tryResetEncoders();
            }

            // Presets
            if (gamepad2.b) {
                joints.terminateActivePreset();
            } else if (gamepad2.dpad_down) {
                joints.activatePreset(SamJoints.Pose.ARENA);
            } else if (gamepad2.dpad_left) {
                joints.activatePreset(SamJoints.Pose.RAIL);
            } else if (gamepad2.dpad_up) {
                joints.activatePreset(SamJoints.Pose.HIGHBAR);
            } else if (gamepad2.y) {
                joints.activatePreset(SamJoints.Pose.BASKET_LOW);
            } else if (gamepad2.dpad_right) {
                joints.activatePreset(SamJoints.Pose.RAIL_UP);
            } else if (gamepad2.back) {
                joints.activatePreset(SamJoints.Pose.PARKED);
            } else {
                joints.stepActivePreset();
            }

            // Individual joint control
            double basePower   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative stick_y value
            double armPower    = -gamepad1.right_stick_y; // Note: pushing stick forward gives negative stick_y value
            double wristPower  = (-gamepad1.left_trigger + gamepad1.right_trigger);
            if (Math.abs(basePower)>0 || Math.abs(armPower)>0 || Math.abs(wristPower)>0) {
                joints.terminateActivePreset();
            }
            if (!joints.isPresetActive()) {
                joints.actuate(basePower, armPower, wristPower);
            }

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            joints.addTelemetry();
            if (joints.isFullyCalibrated()) {
                telemetry.addData("#", "*** FULLY CALIBRATED ***");
                telemetry.addData(">", "Parked Pose: BACK");
                telemetry.addData(">", "Arena Pose: DPAD_DOWN");
                telemetry.addData(">", "Rail Pose: DPAD_LEFT");
                telemetry.addData(">", "Highbar Pose: DPAD_UP");
                telemetry.addData(">", "Basket Pose: Y");
            }
            telemetry.update();
        }
    }
}
