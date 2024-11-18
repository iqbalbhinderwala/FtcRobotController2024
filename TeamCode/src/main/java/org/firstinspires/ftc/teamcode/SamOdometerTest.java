package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

@TeleOp(name="[Sam] Odometer Test", group="SamTest")
@Disabled
public class SamOdometerTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor odometerX      = null;
    private DcMotor odometerY      = null;

    DcMotor mainMotor = null;

    double maxPower = 0.5;
    private int target = 0;
    private int distance = 10;

    private ElapsedTime lastPress = new ElapsedTime();
    private final double PRESS_DELAY = 0.25;

    // https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/
    private static final double GOBILDA_ODOMETRY_POD_DIAMETER_48MM = 48;

    // goBilda GripForce™ Mecanum Wheel Set (Ø104mm)
    // https://www.gobilda.com/gripforce-mecanum-wheel-set-o104mm-40a-durometer-rollers/
    private static final double GOBILDA_MECHANUM_WHEEL_DIAMETER_104MM = 104;

    private static final double WHEEL_DIAMETER_MM = GOBILDA_MECHANUM_WHEEL_DIAMETER_104MM;
    private static final double ENCODER_COUNT_PER_REVOLUTION = 81.1;

    // private static final double WHEEL_DIAMETER_MM = 48;
    // private static final double ENCODER_COUNT_PER_REVOLUTION = 2000;

    private static final double MM_PER_COUNT = (WHEEL_DIAMETER_MM * Math.PI) / ENCODER_COUNT_PER_REVOLUTION;
    private static final double INCH_PER_COUNT = MM_PER_COUNT / 25.4;
    private static final double COUNT_PER_INCH_STRAIGHT = 25.4 / MM_PER_COUNT;

    boolean isForwardDirectionInverted = false;

    private void initMotors() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor 1");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "motor 2");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor 4");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "motor 3");
        odometerX = hardwareMap.get(DcMotor.class, "odometer drive");
        odometerY = hardwareMap.get(DcMotor.class, "odometer strafe");

        leftFrontDrive .setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive  .setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive .setDirection(DcMotor.Direction.REVERSE);
        odometerX.setDirection(DcMotor.Direction.REVERSE); // +-ve forward
        odometerY.setDirection(DcMotor.Direction.FORWARD); // +-ve left

        leftFrontDrive .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void toggleForwardDirection() {
        isForwardDirectionInverted = !isForwardDirectionInverted;

        // Swap leftFrontDrive <-> rightBackDrive
        DcMotor temp = leftFrontDrive;
        leftFrontDrive = rightBackDrive;
        rightBackDrive = temp;

        // Swap leftBackDrive <-> rightFrontDrive
        temp = leftBackDrive;
        leftBackDrive = rightFrontDrive;
        rightFrontDrive = temp;

        // Invert motor / odometer encoder direction
        for (DcMotor motor : Arrays.asList(odometerX, odometerY,
                leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive))
        {
            if (motor.getDirection() == DcMotor.Direction.FORWARD) {
                motor.setDirection(DcMotor.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotor.Direction.FORWARD);
            }
        }
    }

    public void setModeForAll(DcMotor.RunMode mode) {
        leftFrontDrive .setMode(mode);
        leftBackDrive  .setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive .setMode(mode);
        odometerX.setMode(mode);
        odometerY.setMode(mode);
    }

    public void setPowerForAll(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    public void setTargetForAll(int target) {
        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);
    }

    public void driveByPower(int target, double maxPower) {
        setModeForAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeForAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPowerForAll(maxPower); // start motors
        while (opModeIsActive() && mainMotor.getCurrentPosition() < target && !gamepad1.b) {
            addCommonTelemetry();
            telemetry.update();
        }
        Log.d(TAG, mainMotor.getMode()+" "+mainMotor.getZeroPowerBehavior() +
                " Stopping motors at target " + mainMotor.getCurrentPosition());
        setPowerForAll(0); // stop motors
    }

    public void driveBySpeed(int target, double maxPower) {
        setModeForAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeForAll(DcMotor.RunMode.RUN_USING_ENCODER);
        setPowerForAll(maxPower); // start motors
        while (opModeIsActive() && mainMotor.getCurrentPosition() < target && !gamepad1.b) {
            addCommonTelemetry();
            telemetry.update();
        }
        Log.d(TAG, mainMotor.getMode()+" "+mainMotor.getZeroPowerBehavior() +
                " Stopping motors at target " + mainMotor.getCurrentPosition());
        setPowerForAll(0); // stop motors
    }

    public void driveByTarget(int target, double maxPower) {
        setModeForAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetForAll(target);
        setModeForAll(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerForAll(maxPower); // start motors
        while (opModeIsActive() && mainMotor.getCurrentPosition() < target && !gamepad1.b) {
            addCommonTelemetry();
            telemetry.update();
        }
        Log.d(TAG, mainMotor.getMode()+" "+mainMotor.getZeroPowerBehavior() +
                " Stopping motors at target " + mainMotor.getCurrentPosition());
        setPowerForAll(0); // stop motors
        setModeForAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {
        initMotors();
        toggleForwardDirection();

        mainMotor = leftFrontDrive;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            target = (int) Math.round(distance * COUNT_PER_INCH_STRAIGHT);

            if (gamepad1.left_bumper && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                toggleForwardDirection();
            }
            // Modes
            if (gamepad1.a && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                toggleForwardDirection();
                driveByPower(target, maxPower);
            }
            if (gamepad1.x && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                toggleForwardDirection();
                driveBySpeed(target, maxPower);
            }
            if (gamepad1.y && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                toggleForwardDirection();
                driveByTarget(target, maxPower);
            }

            // MaxPower
            if (gamepad1.dpad_up && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                maxPower += 0.1;
            }
            if (gamepad1.dpad_down && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                maxPower -= 0.1;
            }
            maxPower = Range.clip(maxPower, 0, 1);

            // Distance
            if (gamepad1.dpad_left && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                if (distance <= 5) {
                    distance -= 1;
                } else {
                    distance -= 5;
                }
            }
            if (gamepad1.dpad_right && lastPress.seconds() > PRESS_DELAY) {
                lastPress.reset();
                if (distance < 5) {
                    distance += 1;
                } else {
                    distance += 5;
                }
            }
            // Show the elapsed game time and wheel power.
            addCommonTelemetry();
            telemetry.addData(">", "----------------------");
            telemetry.addData(">", "Y - Drive by TARGET (RUN_TO_POSITION)");
            telemetry.addData(">", "X - Drive by SPEED (RUN_WITH_ENCODER)");
            telemetry.addData(">", "A - Drive by POWER (RUN_WITHOUT_ENCODER)");
            telemetry.addData(">", "B - Force Stop");
            telemetry.addData(">", "L BUMPER - Toggle Direction");
            telemetry.addData(">", "DPAD U/D - Power");
            telemetry.addData(">", "DPAD L/R - Distance");
            telemetry.update();
        }
    }

    private void addCommonTelemetry() {
        telemetry.addData("Forward", !isForwardDirectionInverted);
        telemetry.addData("Mode", mainMotor.getMode());
        telemetry.addData("Max power", maxPower);
        telemetry.addData("Distance", distance);
        telemetry.addData("", "Position %d (%.1f in)",
                mainMotor.getCurrentPosition(), mainMotor.getCurrentPosition()*INCH_PER_COUNT);
        telemetry.addData("", "Target %d  (%d in)", target, distance);
        int deltaTarget = mainMotor.getCurrentPosition() - target;
        telemetry.addData("", "deltaTarget %d  (%.1f in)", deltaTarget, deltaTarget*INCH_PER_COUNT);
    }

    static final String TAG = "SAM_WHEELS_CALIB::"; // Define your tag
}
