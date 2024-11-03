package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SamJoints {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    /* Declare Component members. */

    // Motors
    private DcMotor baseMotor = null;
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;

    // Reference Point Sensors
    private TouchSensor baseSensor = null;
    private TouchSensor armSensor = null;
    private TouchSensor wristSensor = null;

    boolean isBaseCalibrated = false;
    boolean isArmCalibrated = false;
    boolean isWristCalibrated = false;

    private Pose activePreset = Pose.NONE;

    public SamJoints(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    // Initialize all hardware components.
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        baseMotor  = opMode.hardwareMap.get(DcMotor.class, "motor 5");
        armMotor   = opMode.hardwareMap.get(DcMotor.class, "motor 6");
        wristMotor = opMode.hardwareMap.get(DcMotor.class, "motor 7");

        // +ve raise up; -ve lower towards ground
        baseMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        baseSensor  = opMode.hardwareMap.get(TouchSensor.class, "base sensor");
        armSensor   = opMode.hardwareMap.get(TouchSensor.class, "arm sensor");
        wristSensor = opMode.hardwareMap.get(TouchSensor.class, "wrist sensor");

        // We do not automatically reset the encoders in case they are already calibrated
        // relative to the reference position given by the optical sensors.
        baseMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the motors to BRAKE mode
        baseMotor .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Start all hardware components.
    public void start(){
        tryResetEncoders();
    }

    public void stepActivePreset() {
        // Terminate active preset if motors stopped
        if (activePreset != Pose.NONE && !areMotorsBusy()) {
            terminateActivePreset();
        }
    }

    public void addTelemetry(){
        opMode.telemetry.addData("Current position", "Base::%d, Arm::%d, Wrist::%d",
                baseMotor.getCurrentPosition(), armMotor.getCurrentPosition(), wristMotor.getCurrentPosition());
        opMode.telemetry.addData("Motor power", "Base::%.1f, Arm::%.1f, Wrist::%.1f",
                baseMotor.getPower(), armMotor.getPower(), wristMotor.getPower());
        opMode.telemetry.addData("Motor busy", "Base::%b, Arm::%b, Wrist::%b",
                baseMotor.isBusy(), armMotor.isBusy(), wristMotor.isBusy());
        opMode.telemetry.addData("Motor sensor", "Base::%b, Arm::%b, Wrist::%b",
                baseSensor.isPressed(), armSensor.isPressed(), wristSensor.isPressed());
        opMode.telemetry.addData("Calibrated", "Base::%b, Arm::%b, Wrist::%b",
                isBaseCalibrated, isArmCalibrated, isWristCalibrated);
        opMode.telemetry.addData("IsForward / Extended", "Base::%b, Arm::%b",
                isBaseForward(), isArmExtended());
        opMode.telemetry.addData("Active Preset Target", activePreset);
    }

    void setBaseMotorTargetPostion(int targetPos, double maxPower){
        stopMotor(baseMotor);
        startMotorTargetPosition(baseMotor, targetPos, maxPower);

        opMode.telemetry.addData("baseMotorTarget", targetPos);
        opMode.telemetry.addData("current base pos", baseMotor.getCurrentPosition());

    }

    private boolean isBaseForward() {
            return isBaseCalibrated && (baseMotor.getCurrentPosition() > BASE_POS_FORWARD_MIN);
    }

    private boolean isArmExtended() {
        return isArmCalibrated && (armMotor.getCurrentPosition() > ARM_POS_EXTENDED_MIN);
    }

    public void actuate(double basePower, double armPower, double wristPower) {
        int basePos  = baseMotor.getCurrentPosition();
        int armPos   = armMotor.getCurrentPosition();
        int wristPos = wristMotor.getCurrentPosition();

        // If moving past safe limits, cut the power
        if (isBaseCalibrated) {
            basePower = cutPowerIfMovingPastLimits(basePower, basePos, BASE_POS_MIN, BASE_POS_MAX);
        }
        if (isArmCalibrated) {
            armPower = cutPowerIfMovingPastLimits(armPower, armPos, ARM_POS_MIN, ARM_POS_MAX);
        }
        if (isWristCalibrated) {
            wristPower = cutPowerIfMovingPastLimits(wristPower, wristPos, WRIST_POS_MIN, WRIST_POS_MAX);
        }

        // Limit Base Forward | Arm Extension
        if (isBaseForward()) {
            armPower = cutPowerIfMovingPastLimits(armPower, armPos, ARM_POS_MIN, ARM_POS_EXTENDED_MIN);
        }
        if (isArmExtended()) {
            basePower = cutPowerIfMovingPastLimits(basePower, basePos, BASE_POS_MIN, BASE_POS_FORWARD_MIN);
        }

        double maxBasePower  = isBaseCalibrated  ? BASE_RUN_POWER  : BASE_SEARCH_POWER;
        double maxArmPower   = isArmCalibrated   ? ARM_RUN_POWER   : ARM_SEARCH_POWER;
        double maxWristPower = isWristCalibrated ? WRIST_RUN_POWER : WRIST_SEARCH_POWER;

        baseMotor.setPower(Range.clip(basePower, -maxBasePower, maxBasePower));
        armMotor.setPower(Range.clip(armPower, -maxArmPower, maxArmPower));
        wristMotor.setPower(Range.clip(wristPower, -maxWristPower, maxWristPower));
    }

    private double cutPowerIfMovingPastLimits(double power, int pos, int minPos, int maxPos) {
        if ((pos < minPos && power < 0) || (pos > maxPos && power > 0))
            return 0;
        return power;
    }

    private void activatePose(Pose pose, int basePos, int armPose, int wristPos) {
        if (!isFullyCalibrated() || pose == activePreset) {
            return;
        }
        // Terminate previous active preset
        terminateActivePreset();
        // Set new active preset
        activePreset = pose;
        // Start the motors
        startMotorTargetPosition(wristMotor, wristPos, WRIST_RUN_POWER);
        startMotorTargetPosition(baseMotor,  basePos,  BASE_RUN_POWER);
        startMotorTargetPosition(armMotor,   armPose,  ARM_RUN_POWER);
    }

    public boolean isPresetActive() {
        return activePreset != Pose.NONE;
    }

    public void terminateActivePreset() {
        if (activePreset != Pose.NONE) {
            // Stop the motors
            stopMotor(baseMotor);
            stopMotor(armMotor);
            stopMotor(wristMotor);
            // Reset active preset
            activePreset = Pose.NONE;
        }
    }

    private boolean areMotorsBusy() {
        return  baseMotor.isBusy() || armMotor.isBusy() || wristMotor.isBusy();
    }

    private void runMotorFromCurrent(DcMotor motor, int delta, double maxPower) {
        int targetPos = motor.getCurrentPosition() + delta;
        runMotorToPosition(motor, targetPos, maxPower);
    }

    private void runMotorToPosition(DcMotor motor, int targetPos, double maxPower) {
        // Start the motor
        startMotorTargetPosition(motor, targetPos, maxPower);
        // Keep looping while we are still active, and motor is running.
        while(opMode.opModeIsActive() && motor.isBusy()) {
            opMode.sleep(CYCLE_MS);
        }
        // Stop the motor
        stopMotor(motor);
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

    private boolean searchReferencePosition(
            DcMotor motor,
            TouchSensor sensor,
            int searchDelta,        // search from current pos
            double maxPower,        // 0..1
            double timeout)         // seconds
    {
        if (sensor.isPressed()) // Already at reference position
            return true;

        // Determine new target position, and pass to motor controller
        int startPos = motor.getCurrentPosition();
        int targetPos = startPos + searchDelta;

        int[] edgePos = new int[]{0, 0};
        int edgeIndex = 0;
        boolean edgeTarget = true; // First sensor value to look for

        ElapsedTime searchTime = new ElapsedTime();
        searchTime.reset();

        // Set Target FIRST, then turn on RUN_TO_POSITION, with the required motor speed
        startMotorTargetPosition(motor, targetPos, maxPower);

        // Keep looping while we are still active, and motor is running.
        while(opMode.opModeIsActive() && motor.isBusy()) {
            int curPos = motor.getCurrentPosition();
            boolean curState = sensor.isPressed();
            double speed = (curPos - startPos) / searchTime.seconds();

            // Sensor target
            if (edgeIndex < 2 && curState == edgeTarget) { // Found an edge
//                telemetry.addData(">", "%.1f @ %d = %b  SPD=%.1f/s @ %.2fPWR",
//                        searchTime.seconds(), curPos, curState,
//                        speed, motor.getPower());

                edgePos[edgeIndex] = curPos;
                opMode.telemetry.addData(">", "Goal Reached @%d = %b", edgePos[edgeIndex], edgeTarget);

                // Invert target for the next edge
                edgeTarget = !edgeTarget;
                ++edgeIndex;

                // Found both edges
                if (edgeIndex == 2) {
                    targetPos = (edgePos[0] + edgePos[1])/2;
                    motor.setTargetPosition(targetPos);
                    opMode.telemetry.addData(">", "HOLE SIDES @ [%d , %d] Width =  %d",
                            edgePos[0], edgePos[1], Math.abs(edgePos[1]-edgePos[0]));
                }
            }
            // Emergency Stop
            if (opMode.gamepad1.b) {
                opMode.telemetry.addLine("Emergency Stop");
                speed = (motor.getCurrentPosition() - startPos) / searchTime.seconds();
                opMode.telemetry.addData(">", "%.1f @ %d = %b  SPD=%.1f/s @ %.2fPWR",
                        searchTime.seconds(), motor.getCurrentPosition(), sensor.isPressed(),
                        speed, motor.getPower());
                opMode.telemetry.addLine("PRESS DPAD_RIGHT TO CONTINUE");
                opMode.telemetry.update();
                while (opMode.opModeIsActive() && !opMode.gamepad1.dpad_right) opMode.sleep(100);
                break;
            }
            // Timeout
            if (searchTime.seconds() > timeout) {
                opMode.telemetry.addData(">", "Timeout %.1f sec", searchTime.seconds());
                break;
            }
        }

        opMode.telemetry.addData(">", "%.1f sec, LOOP EXIT(opActive=%b motorBusy=%b)",
                searchTime.seconds(), opMode.opModeIsActive(), motor.isBusy());
        opMode.telemetry.addData("Power", "%.2f", motor.getPower());

        stopMotor(motor);

        boolean success = sensor.isPressed();
        opMode.telemetry.addData("-------->", success ? "SUCCESS" : "SEARCH FAILED");
        opMode.telemetry.addData(">DONE CALIB", "STARTED @%d  STOPPED @%d", startPos, motor.getCurrentPosition());
//        telemetry.addLine("PRESS DPAD_RIGHT TO CONTINUE");
//        telemetry.update(); while (!gamepad1.dpad_right) sleep(100);

        if (success){
            tryResetEncoders();
        }
        return success;
    }

    public boolean isFullyCalibrated() {
        return isArmCalibrated && isWristCalibrated;
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
        if (!isWristCalibrated && wristSensor.isPressed())
        {
            wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (wristSensor.isPressed()) {
                isWristCalibrated = true;
            }
        }
        return (baseSensor.isPressed() && armSensor.isPressed() && wristSensor.isPressed());
    }

    public enum Pose {
        NONE,
        PARKED,
        ARENA,
        HIGHBAR,
        LOWBAR,
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
                activatePose(pose, 7750, 1600, 3500);
                  break;
            case HIGHBAR:
                activatePose(pose, 4575, 8000, 8000);
                break;
            case LOWBAR:
                activatePose(pose, 4500, 4000, 7000);
                break;
            case TRANSITION:
                activatePose(pose, BASE_POS_FORWARD_MIN, ARM_POS_EXTENDED_MIN, 5000);
                break;
        }
    }

    // BASE MOTOR
    final int    BASE_POS_MAX         = +7750; // MAX USER
    final int    BASE_POS_FORWARD_MIN = +5300;
//    final int    BASE_POS_VERTICAL    = +3200;

    final int    BASE_POS_MIN         =    +0; // MIN USER
    //    final int    BASE_SENSOR_SPAN     =   700;
    final double BASE_SEARCH_POWER    =   0.5;
    final double BASE_RUN_POWER       =   1.0;

    // ARM MOTOR
    final int    ARM_POS_MAX          = +8300; // MAX USER
//    final int    ARM_POS_EXTENDED_MAX = +6200;
//    final int    ARM_POS_90DEG        = +3900;
    final int    ARM_POS_EXTENDED_MIN = +1600;
    final int    ARM_POS_MIN          =    +0; // MIN USER

    //    final int    ARM_SENSOR_SPAN      =   450;
    final double ARM_SEARCH_POWER     =   0.5;
    final double ARM_RUN_POWER        =   1.0;

    // WRIST MOTOR
    final int    WRIST_POS_MAX        =  +8500; // MAX USER
    final int    WRIST_POS_MIN        =     +0; // MIN USER
//    final int    WRIST_SENSOR_SPAN    =    ?;
    final double WRIST_SEARCH_POWER   =    0.5;
    final double WRIST_RUN_POWER      =    1.0;

    static final int CYCLE_MS = 15;     // period of each cycle
}
