package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SamJoints {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    /* Declare Component members. */

    // Motors
    private DcMotor baseMotor = null;
    private DcMotor armMotor = null;
    private Servo wristServo = null;

    // Reference Point Sensors
    private TouchSensor baseSensor = null;
    private TouchSensor armSensor = null;

    boolean isBaseCalibrated = false;
    boolean isArmCalibrated = false;

    private Pose activePreset = Pose.NONE;

    public SamJoints(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    // Initialize all hardware components.
    public void init() {
        initJointsHardware(opMode.hardwareMap);

        // We do not automatically reset the encoders in case they are already calibrated
        // relative to the reference position given by the optical sensors.
        baseMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the motors to BRAKE mode
        baseMotor .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Start all hardware components.
    public void start(){
        tryResetEncoders();
    }

    private void initJointsHardware(HardwareMap hardwareMap) {
        baseSensor  = hardwareMap.get(TouchSensor.class, "base sensor");
        armSensor   = hardwareMap.get(TouchSensor.class, "arm sensor");
        baseMotor   = hardwareMap.get(DcMotor.class, "motor base");
        armMotor    = hardwareMap.get(DcMotor.class, "motor arm");
        wristServo  = hardwareMap.get(Servo.class, "servo wrist");

        baseMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor .setDirection(DcMotor.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.REVERSE); // 0: parked
    }

    public void addTelemetry(){
        opMode.telemetry.addData("Current position", "Base::%d, Arm::%d, Wrist::%.2f",
                baseMotor.getCurrentPosition(), armMotor.getCurrentPosition(), wristServo.getPosition());
        opMode.telemetry.addData("Motor power", "Base::%.1f, Arm::%.1f",
                baseMotor.getPower(), armMotor.getPower());
        opMode.telemetry.addData("Motor busy", "Base::%b, Arm::%b",
                baseMotor.isBusy(), armMotor.isBusy());
        opMode.telemetry.addData("Motor sensor", "Base::%b, Arm::%b",
                baseSensor.isPressed(), armSensor.isPressed());
        opMode.telemetry.addData("Calibrated", "Base::%b, Arm::%b",
                isBaseCalibrated, isArmCalibrated);
        opMode.telemetry.addData("IsForward / Extended", "Base::%b, Arm::%b",
                isBaseForward(), isArmExtended());
        opMode.telemetry.addData("Active Preset Target", activePreset);
    }

    private boolean isBaseForward() {
        return isBaseCalibrated && (baseMotor.getCurrentPosition() > BASE_POS_FORWARD_MIN);
    }

    private boolean isArmExtended() {
        return isArmCalibrated && (armMotor.getCurrentPosition() > ARM_POS_EXTENDED_MIN);
    }

    public void actuate(double basePower, double armPower, double wristPower) {
        int basePos = baseMotor.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        double wristPos = wristServo.getPosition();

        // If moving past safe limits, cut the power
        if (isBaseCalibrated && isMovingPastLimits(basePower, basePos, BASE_POS_MIN, BASE_POS_MAX)) {
            basePower = 0;
        }
        if (isArmCalibrated && isMovingPastLimits(armPower, armPos, ARM_POS_MIN, ARM_POS_MAX)) {
            armPower = 0;
        }

        // Limit Base Forward | Arm Extension
        if (isBaseForward() && isMovingPastLimits(armPower, armPos, ARM_POS_MIN, ARM_POS_EXTENDED_MIN)) {
            // If base is forward, restrict arm from extending
            armPower = 0;
        }
        if (isArmExtended() && isMovingPastLimits(basePower, basePos, BASE_POS_MIN, BASE_POS_FORWARD_MIN)) {
            // If arm is extended, restrict base being forward
            basePower = 0;
        }

        double maxBasePower  = isBaseCalibrated ? BASE_RUN_POWER : BASE_SEARCH_POWER;
        double maxArmPower   = isArmCalibrated  ? ARM_RUN_POWER  : ARM_SEARCH_POWER;

        // Also reduce power when approaching ZERO to match search power used when calibrating
        if (isBaseCalibrated && isApproachingZero(basePower, basePos, BASE_SENSOR_SPAN)) {
            maxBasePower = BASE_SEARCH_POWER;
        }
        if (isArmCalibrated && isApproachingZero(armPower, armPos, ARM_SENSOR_SPAN)) {
            maxArmPower = ARM_SEARCH_POWER;
        }

        baseMotor.setPower(Range.clip(basePower, -maxBasePower, maxBasePower));
        armMotor .setPower(Range.clip(armPower,  -maxArmPower,  maxArmPower));
        wristServo.setPosition(wristPos + wristPower * WRIST_INCREMENT);
    }

    private boolean isMovingPastLimits(double power, int pos, int minPos, int maxPos) {
        return (pos < minPos && power < 0) || (pos > maxPos && power > 0);
    }

    private boolean isApproachingZero(double power, int pos, int radius) {
        return (Math.abs(pos) < radius) && (power*pos < 0);
    }

    private void activatePose(Pose pose, int basePos, int armPose, double wristPos) {
        if (!isFullyCalibrated() || pose == activePreset) {
            return;
        }
        // Terminate previous active preset
        terminateActivePreset();
        // Set new active preset
        activePreset = pose;
        // Start the motors
        wristServo.setPosition(wristPos);
        startMotorTargetPosition(baseMotor,  basePos,  BASE_RUN_POWER);
        startMotorTargetPosition(armMotor,   armPose,  ARM_RUN_POWER);
    }

    public boolean isPresetActive() {
        return activePreset != Pose.NONE;
    }

    public void stepActivePreset() {
        // Terminate active preset if motors stopped
        if (activePreset != Pose.NONE && !areMotorsBusy()) {
            terminateActivePreset();
        } else {
            // Also reduce power when approaching ZERO to match search power used when calibrating
            double basePower = baseMotor.getPower();
            int basePos = baseMotor.getCurrentPosition();
            if (isBaseCalibrated && isApproachingZero(basePower, basePos, BASE_SENSOR_SPAN)) {
                double maxBasePower = BASE_SEARCH_POWER;
                baseMotor.setPower(Range.clip(basePower, -maxBasePower, maxBasePower));
            }
            double armPower = armMotor.getPower();
            int armPos = armMotor.getCurrentPosition();
            if (isArmCalibrated && isApproachingZero(armPower, armPos, ARM_SENSOR_SPAN)) {
                double maxArmPower = ARM_SEARCH_POWER;
                armMotor .setPower(Range.clip(armPower,  -maxArmPower,  maxArmPower));
            }
        }
    }

    public void terminateActivePreset() {
        if (activePreset != Pose.NONE) {
            // Stop the motors
            stopMotor(baseMotor);
            stopMotor(armMotor);
            // Reset active preset
            activePreset = Pose.NONE;
        }
    }

    private boolean areMotorsBusy() {
        return baseMotor.isBusy() || armMotor.isBusy();
    }

    private void startMotorTargetPosition(DcMotor motor, int targetPos, double maxPower) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        motor.setTargetPosition(targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required motor speed (must be positive for RUN_TO_POSITION)
        motor.setPower(Math.abs(maxPower));
    }

    private void stopMotor(DcMotor motor) {
        // NOTE: Wrist Motor was difficult to get it to stop.
        // Steps below may not be optimal, but was the first combination found to work.
        // TODO: Understand how to properly stop a motor in RunMode.RUN_TO_POSITION
        motor.setPower(0);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public boolean isFullyCalibrated() {
        return isBaseCalibrated && isArmCalibrated;
    }

    public boolean tryResetEncoders(){
        if (!isBaseCalibrated && baseSensor.isPressed())
        {
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (baseSensor.isPressed()) {
                isBaseCalibrated = true;
            }
        }
        if (!isArmCalibrated && armSensor.isPressed())
        {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (armSensor.isPressed()) {
                isArmCalibrated = true;
            }
        }
        return baseSensor.isPressed() && armSensor.isPressed();
    }

    public enum Pose {
        NONE,
        PARKED,
        ARENA,
        HIGHBAR,
        LOWBAR,
        RAIL,
        RAIL_UP,
        TRANSITION,
    }

    public void activatePreset(Pose pose)
    {
        switch (pose) {
            case NONE:
                terminateActivePreset();
                break;
            case PARKED:
                activatePose(pose, 0, 0, 0);
                break;
            case ARENA:
                activatePose(pose, 8450, 1050, 0.65);
                activatePose(pose, 8450, 1050, 0.65);
                  break;
            case HIGHBAR:
                activatePose(pose, 4756, 2325, 0.50);
                break;
            case LOWBAR:
//                activatePose(pose, 4500, 4000, 7000);
                break;
            case RAIL:
                activatePose(pose, 6450, 0, 0.65);
                activatePose(pose, 6450, 0, 0.65);
                break;
            case RAIL_UP:
                activatePose(pose, 6450-1500, 0, 0.65);
                break;
            case TRANSITION:
//                activatePose(pose, BASE_POS_FORWARD_MIN, ARM_POS_EXTENDED_MIN, 5000);
                break;
        }
    }

    // BASE MOTOR
    final int    BASE_POS_MAX         = +8500; // MAX USER
    final int    BASE_POS_FORWARD_MIN = +6000;
//    final int    BASE_POS_VERTICAL    = +3200;

    final int    BASE_POS_MIN         =    +0; // MIN USER
    final int    BASE_SENSOR_SPAN     =   700;
    final double BASE_SEARCH_POWER    =   0.5;
    final double BASE_RUN_POWER       =   1.0;

    // ARM MOTOR
    final int    ARM_POS_MAX          = +8300; // MAX USER
//    final int    ARM_POS_90DEG        = +3900;
    final int    ARM_POS_EXTENDED_MIN = +2000;
    final int    ARM_POS_MIN          =    +0; // MIN USER

    final int    ARM_SENSOR_SPAN      =   450;
    final double ARM_SEARCH_POWER     =   0.5;
    final double ARM_RUN_POWER        =   1.0;

    // WRIST SERVO
    final double WRIST_INCREMENT      = 0.015;

}
