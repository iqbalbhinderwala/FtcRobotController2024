package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SamEncoderDriveTrain
{
    /* Reference to the parent OpMode. */
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    boolean isForwardDirectionInverted = false;

    /* Declare Component members. */
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static double diameter_mm = 76;
    private static double CperRev = 81;
    private static double pi = 3.1415926;
    private static double MM_per_Count = (diameter_mm*pi)/CperRev;
    private static double IN_per_Count = MM_per_Count/25.4;
    private static double Counts_per_IN = 1/IN_per_Count;
    private static double Counts_per_IN_Strafe = 16;

    private boolean isBackupHardwareConfig = false;

    // Constructor
    public SamEncoderDriveTrain(LinearOpMode myOpMode, boolean backupHardwareConfig) {
        opMode = myOpMode;
        isBackupHardwareConfig = backupHardwareConfig;
    }

    public void init() {
        initMotors();
    }

    public void swapForwardDirection() {
        isForwardDirectionInverted = !isForwardDirectionInverted;
        initMotors();
    }

    private void toggleMotorDirection(DcMotor motor){
        if (motor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private void initMotors() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = opMode.hardwareMap.get(DcMotor.class, "motor 1");
        leftBackDrive   = opMode.hardwareMap.get(DcMotor.class, "motor 2");
        rightFrontDrive = opMode.hardwareMap.get(DcMotor.class, "motor 4");
        rightBackDrive  = opMode.hardwareMap.get(DcMotor.class, "motor 3");

        leftFrontDrive .setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive  .setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive .setDirection(DcMotor.Direction.REVERSE);

        // HACK - FOR SOME REASON THE SECOND ROBOT HAS LEFT_BACK WHEEL REVERSED
        if (isBackupHardwareConfig) {
            toggleMotorDirection(leftBackDrive);
        }

        if (isForwardDirectionInverted) {
            toggleMotorDirection(leftFrontDrive);
            toggleMotorDirection(leftBackDrive);
            toggleMotorDirection(rightFrontDrive);
            toggleMotorDirection(rightBackDrive);
        }

        leftFrontDrive .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int target = 0;
        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveByDistance(double distanceIN, boolean strafe, double power) {
        int target = 0;
        
        if (strafe){
            target = (int)Math.round(distanceIN * Counts_per_IN_Strafe);
            leftFrontDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(-target);
            rightFrontDrive.setTargetPosition(-target);
            rightBackDrive.setTargetPosition(target);
        } else {
            target = (int)Math.round(distanceIN * Counts_per_IN);
            leftFrontDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(target);
            rightFrontDrive.setTargetPosition(target);
            rightBackDrive.setTargetPosition(target);
        }

        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);

        while(opMode.opModeIsActive() ) {
            if (Math.abs(leftFrontDrive.getCurrentPosition()-target) < 3) {
                break;
            }
            opMode.telemetry.addData("MotorTarget", "%d", target);
            opMode.telemetry.addData("MotorPosition", "%d", leftFrontDrive.getCurrentPosition());
            opMode.telemetry.update();
            opMode.sleep(CYCLE_MS);
        }

        target = 0;
        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    static final int CYCLE_MS = 15;     // period of each cycle
}
