package org.firstinspires.ftc.teamcode.Vex.Hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class VexActuators {
    private LinearOpMode opMode;

    // Declare OpMode members.
    private DcMotor intakeMotor = null;
    private DcMotorEx topShooterMotor = null;
    private DcMotorEx bottomShooterMotor = null;
    private Servo firstGateServo = null;
    private Servo secondGateServo = null;
    private VoltageSensor voltageSensor = null;

    private double shooterTargetRPM = 0.0;
    private double shooterPower = 0.0;
    private ElapsedTime shooterSpinupTimer = new ElapsedTime();


    public VexActuators(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables.
        intakeMotor = hardwareMap.get(DcMotor.class, "m2");
        topShooterMotor = hardwareMap.get(DcMotorEx.class, "m0");
        bottomShooterMotor = hardwareMap.get(DcMotorEx.class, "m1");

        firstGateServo = hardwareMap.get(Servo.class, "gate a");
        secondGateServo = hardwareMap.get(Servo.class, "gate b");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        topShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterSpinupTimer = new ElapsedTime();
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void setShooterPower(double power) {
        // Reset shooter timer on shooter starting up
        if (shooterPower == 0 && power != 0) {
            shooterSpinupTimer.reset();
        }
        power = Range.clip(power, -1, 1);
        shooterPower = power;
        topShooterMotor.setPower(power);
        bottomShooterMotor.setPower(power);
    }

    public double getShooterPower() {
        return shooterPower;
    }

    public double getShooterTicksPerSecond() {
        return (topShooterMotor.getVelocity() + bottomShooterMotor.getVelocity()) / 2.0;
    }

    public double getShooterRPM() {
        return getShooterTicksPerSecond() / SHOOTER_TICKS_PER_REVOLUTION * 60.0;
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public void openGateA() {
        firstGateServo.setPosition(GATE_A_OPEN);
    }

    public void closeGateA() {
        firstGateServo.setPosition(GATE_A_CLOSED);
    }

    public void openGateB() {
        secondGateServo.setPosition(GATE_B_OPEN);
    }

    public void closeGateB() {
        secondGateServo.setPosition(GATE_B_CLOSED);
    }

    public double getGateAPosition() {
        return firstGateServo.getPosition();
    }

    public double getGateBPosition() {
        return secondGateServo.getPosition();
    }

    public void setShooterRPM(double targetRPM) {
        targetRPM = Range.clip(targetRPM, 0, SHOOTER_RPM_MAX);
        shooterTargetRPM = targetRPM;
        double adjustedPower = calculateAdjustedShooterPowerForTargetRPM(targetRPM);
        setShooterPower(adjustedPower);
    }

    private double calculateAdjustedShooterPowerForTargetRPM(double targetRPM) {
        // Power off the motors if the target is zero.
        if (targetRPM == 0) {
            Log.d(TAG, "calculateAdjustedShooterPowerForTargetRPM: Target RPM is 0, returning power 0.");
            return 0;
        }

        // Get the current average RPM of the shooter motors.
        double currentRPM = getShooterRPM();
        // Calculate the difference between the current RPM and the desired target RPM.
        double error = currentRPM - targetRPM;

        Log.d(TAG, String.format("calculateAdjustedShooterPowerForTargetRPM: Target=%.1f, Current=%.1f, Error=%.1f", targetRPM, currentRPM, error));

        // Check if the absolute error is within an acceptable tolerance.
        if (Math.abs(error) < SHOOTER_RPM_TOLERANCE) {
            // If the RPM is close enough to the target, no change is needed.
            Log.d(TAG, String.format("calculateAdjustedShooterPowerForTargetRPM: RPM is within tolerance. Maintaining current power: %.2f", shooterPower));
            return shooterPower;
        } else {
            // If the error is too large, a correction is needed.
            // Only apply a power correction periodically to avoid unstable oscillations.
            if (shooterPower > 0 && shooterSpinupTimer.seconds() > 0.4) {
                Log.d(TAG, "calculateAdjustedShooterPowerForTargetRPM: RPM outside tolerance. Calculating correction.");
                // Apply a proportional correction to the current power.
                // If the shooter is too slow (error is negative), this increases the power.
                // If the shooter is too fast (error is positive), this decreases the power.
                double adjustedPower = shooterPower * (1.0 - error / targetRPM * 0.25);
                Log.d(TAG, String.format("calculateAdjustedShooterPowerForTargetRPM: Applying correction. Old Power=%.2f, New Power=%.2f", shooterPower, adjustedPower));
                return adjustedPower;
            }
            // If the timer hasn't elapsed, predict the initial power.
            // This branch is often hit when the motors are first spinning up.
            double predictedPower = predictShooterPowerForTargetRPM(targetRPM);
            Log.d(TAG, String.format("calculateAdjustedShooterPowerForTargetRPM: Spin-up phase. Returning predicted power: %.2f", predictedPower));
            return predictedPower;
        }
    }

    /**
     * Checks if the shooter's current RPM is within the acceptable tolerance of the target RPM.
     * This method has NO side effects; it only returns a boolean state.
     * @return true if the shooter is at its target speed, false otherwise.
     */
    public boolean isShooterAtTargetRPM() {
        double error = getShooterRPM() - shooterTargetRPM;
        return Math.abs(error) < SHOOTER_RPM_TOLERANCE;
    }

    /**
     * Predicts the motor power needed to achieve a target RPM, with voltage compensation.
     * This function is based on the quadratic model:
     *   y = 0.1877x^2 + 3.9794x + 1.6133
     * where:
     *   y = Power * Voltage
     *   x = RPM / 1000
     *
     * @param targetRPM The desired rotations per minute for the shooter.
     * @return The calculated motor power (from 0.0 to 1.0) required to reach the target RPM.
     */
    public double predictShooterPowerForTargetRPM(double targetRPM) {
        // If the target RPM is zero or less, the power should be zero.
        if (targetRPM <= 0) {
            return 0.0;
        }

        // Get the current battery voltage to compensate for power level changes.
        double currentVoltage = getVoltage();

        // If voltage is very low, return 0 to prevent division by zero or invalid values.
        if (currentVoltage < 5.0) { // A reasonable minimum voltage threshold.
            return 0.0;
        }

        // Calculate 'x' as defined by the formula (RPM / 1000).
        double x = targetRPM / 1000.0;

        // Calculate 'y' (the Power * Voltage product) using the provided quadratic model.
        double powerVoltageProduct = (0.1877 * x * x) + (3.9794 * x) + 1.6133;

        // Solve for the final power by dividing the result by the current voltage.
        double calculatedPower = powerVoltageProduct / currentVoltage;

        // Clamp the result to the valid motor power range [0.0, 1.0].
        return Range.clip(calculatedPower, 0.0, 1.0);
    }


//    /**
//     * Predicts the motor power needed to achieve a target RPM, with voltage compensation.
//     * This function is based on the linear model:
//     *   RPM = 223.64 * (voltage * power) - 302.06
//     *
//     * By rearranging for power, we get:
//     *   power = (RPM + 302.06) / (223.64 * voltage)
//     *
//     * @param targetRPM The desired rotations per minute for the shooter.
//     * @return The calculated motor power (from 0.0 to 1.0) required to reach the target RPM.
//     */
//    public double predictShooterPowerForTargetRPM(double targetRPM) {
//        // If the target RPM is zero or less, the power should be zero.
//        if (targetRPM <= 0) {
//            return 0.0;
//        }
//
//        // Get the current battery voltage to compensate for power level changes.
//        double currentVoltage = getVoltage();
//
//        // If for some reason voltage is very low, prevent division by zero or invalid values.
//        if (currentVoltage < 5.0) { // A reasonable minimum voltage threshold.
//            return 0.0;
//        }
//
//        // Calculate the required power using the rearranged formula.
//        double calculatedPower = (targetRPM + 302.06) / (223.64 * currentVoltage);
//
//        // Clamp the result to the valid motor power range [0.0, 1.0].
//        return Range.clip(calculatedPower, 0.0, 1.0);
//    }

//    /**
//     * Calculates the shooter power based on the target RPM with voltage compensation.
//     * This method is an alternative to using DcMotorEx.setVelocity() when you want to manage the power calculation manually.
//     * It uses a linear model derived from experimental data to map RPM to power.
//     *
//     * @param targetRPM The desired rotations per minute for the shooter wheels.
//     * @return The calculated motor power, a value between 0.0 and 1.0.
//     */
//    private double predictShooterPowerForTargetRPM(double targetRPM) {
//        if (targetRPM <= 0) {
//            return 0.0;
//        }
//
//        // Get the current battery voltage.
//        double voltage = getVoltage();
//
//        // This scale factor is used to compensate for battery voltage drops.
//        // It increases the target power when the voltage is below 12V
//        // and decreases it when the voltage is above 12V.
//        double scaleRPMFor12V = 1 + (12.0 - voltage) / 12.0; // same as: 2 - V/12
////        double scaleRPMFor12V = 12.0 / voltage; // simpler, slightly more aggressive
//
//        // This formula converts the desired RPM into a motor power value.
//        // It's based on a linear model (y = mx + b) where:
//        // y = shooterPower
//        // m = 1 / SHOOTER_12V_RPM_TO_POWER_GAIN
//        // x = targetShooterRPM * scaleRPMTo12V
//        // b = -SHOOTER_12V_RPM_TO_POWER_OFFSET / SHOOTER_12V_RPM_TO_POWER_GAIN
//        // The result is a power value that should achieve the target RPM at the current voltage.
//        double calculatedPower = (scaleRPMFor12V * targetRPM - SHOOTER_12V_RPM_TO_POWER_OFFSET) / SHOOTER_12V_RPM_TO_POWER_GAIN;
//
//        // Clamp the power to the valid range [0.0, 1.0] for the motor.
//        return Range.clip(calculatedPower, 0.0, 1.0);
//    }

    /**
     * Predicts the target shooter RPM based on the distance from the target.
     * The formula provided is y = -11.636x^3 + 180.9x^2 - 814x + 2556.9,
     * where 'x' is in tile units (1 tile = 24 inches) and 'y' is RPM.
     *
     * @param distanceInInches The distance from the target in inches.
     * @return The calculated target RPM for the shooter.
     */
    public double predictShooterRPMFromDistance(double distanceInInches) {
        // Convert the distance from inches to tile units, as required by the formula.
        double x = distanceInInches / 24.0;

        // Because of the cubic fit, do not extrapolate outside the valid range.
        x = Range.clip(x, 2.9, 7); // tiles

        // Apply the polynomial formula to calculate the target RPM.
        double rpm = -11.636 * Math.pow(x, 3)
                + 180.9 * Math.pow(x, 2)
                - 814 * x
                + 2556.9;

        // It's good practice to ensure the returned value is within a sensible range.
        return Range.clip(rpm, 0, SHOOTER_RPM_MAX);
    }

    /**
     * Calculates the shooter power based on the distance to the alliance corner.
     * The formula is power(x in) = (4.48 / 24 * x + 55.055) / 100.
     * @return The calculated shooter power, a value between 0.0 and 1.0.
     */
    public double calculateDistanceBasedShooterPower(double distanceInInches) {
        double power = (4.48 / 24.0 * distanceInInches + 55.055 - 2.5) / 100.0;
        // Clamp the power to be between 0.0 and 1.0, which is what the motor can take.
        return Math.max(0.0, Math.min(1.0, power));
    }

    public void shootSequence(double shooterPower) {
        // Turn on shooters
        setShooterPower(shooterPower);
        opMode.sleep(1000); // Give motors time to spin up

        long DELAY = 250;
        // Run sequence up to 3 times, can be interrupted by releasing Y
        while (true) {
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // CLOSE GATE 2
            closeGateB(); // Closed
            opMode.sleep(DELAY);
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // OPEN GATE 1
            openGateA(); // Open
            opMode.sleep(DELAY);
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // CLOSE GATE 1
            closeGateA(); // Closed
            opMode.sleep(DELAY);
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // OPEN GATE 2
            openGateB(); // Open
            opMode.sleep(DELAY);
        }

        // Turn off shooters and wait for Y release
        setShooterPower(0);
        openGateA(); // Reset Gate A to open
        closeGateB(); // Reset Gate B to closed
        while (opMode.opModeIsActive() && opMode.gamepad1.y) {
            opMode.idle();
        }
    }

    // Constants
    static public final double GATE_A_OPEN = 0.0;
    static public final double GATE_A_CLOSED = 0.20;
    static public final double GATE_B_OPEN = 0.35;
    static public final double GATE_B_CLOSED = 0.65;

    // Shooter
    static private final double SHOOTER_TICKS_PER_REVOLUTION = 28;
    static private final double SHOOTER_12V_RPM_TO_POWER_OFFSET = -85.0;
    static private final double SHOOTER_12V_RPM_TO_POWER_GAIN = 2100.0;
    static public final double SHOOTER_RPM_MAX = 2000;
    static public final double SHOOTER_RPM_TOLERANCE = 40;

    private static final String TAG = "VEX::Actuators";
}
