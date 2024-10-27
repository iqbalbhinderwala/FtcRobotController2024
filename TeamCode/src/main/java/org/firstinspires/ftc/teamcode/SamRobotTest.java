package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="[Sam] Robot Test", group="Sam")
//@Disabled
public class SamRobotTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare Component members. */
    public SamIMUOmniDriveTrain nav = null;
    public SamJoints joints = null;
    public SamClaw claw = null;

    double MAX_DRIVE_SPEED = 0.3;
    double MAX_STRAFE_SPEED = 0.4;
    double MAX_TURN_SPEED = 0.3;

    ElapsedTime lastPress = new ElapsedTime();
    ElapsedTime lastLeftBumper = new ElapsedTime();
    ElapsedTime lastRightBumper = new ElapsedTime();
    final double BUTTON_DELAY = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        nav = new SamIMUOmniDriveTrain(this, false);
        joints = new SamJoints(this);
        claw = new SamClaw(this);

        // Initialize components.
        nav.init();
        joints.init();
        claw.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Start components.
        joints.start(false);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // GAMEPAD 1 - Navigation

            double drive  = -gamepad1.left_stick_y  * MAX_DRIVE_SPEED;
            double strafe = -gamepad1.left_stick_x  * MAX_STRAFE_SPEED;
            double turn   = -gamepad1.right_stick_x * MAX_TURN_SPEED;
            nav.moveRobot(drive, strafe, turn);

            if (gamepad1.left_stick_button &&  lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                nav.swapForwardDirection();
            }

            // GAMEPAD 2 - ARM CONTROL

            // Claw
            if (gamepad2.left_bumper && lastLeftBumper.seconds() > BUTTON_DELAY) {
                lastLeftBumper.reset();
                claw.toggle();
            }

            // Individual joint control
            double basePower   = -gamepad2.left_stick_y;  // Note: pushing stick forward gives negative stick_y value
            double armPower    = -gamepad2.right_stick_y; // Note: pushing stick forward gives negative stick_y value
            double wristPower  = (-gamepad2.left_trigger + gamepad2.right_trigger);
            joints.actuate(basePower, armPower, wristPower);

            if (!joints.isFullyCalibrated())
            {
                joints.tryResetEncoders();
            }

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            joints.addTelemetry();
            if (joints.isFullyCalibrated()) {
                telemetry.addData("#", "*** FULLY CALIBRATED ***");
            }
            telemetry.update();
        }
    }
}
