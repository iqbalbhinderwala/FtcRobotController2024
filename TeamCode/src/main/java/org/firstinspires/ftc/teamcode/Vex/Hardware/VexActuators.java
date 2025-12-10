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
    private DcMotor intakeMotorA = null;
    private DcMotor intakeMotorB = null;

    private DcMotorEx topShooterMotor = null;
    private DcMotorEx bottomShooterMotor = null;
    private Servo firstGateServo = null;
    private Servo secondGateServo = null;
    private VoltageSensor voltageSensor = null;

    private double shooterPower = 0.0;
    private ElapsedTime shooterSpinupTimer = new ElapsedTime();

    public boolean enablePowerAdjustment = false;
    public double powerAdjustementFactor = 0.05;


    public VexActuators(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables.
        intakeMotorA = hardwareMap.get(DcMotor.class, "m2");
        intakeMotorB = hardwareMap.get(DcMotor.class, "m3");
        topShooterMotor = hardwareMap.get(DcMotorEx.class, "m0");
        bottomShooterMotor = hardwareMap.get(DcMotorEx.class, "m1");

        firstGateServo = hardwareMap.get(Servo.class, "gate a");
        secondGateServo = hardwareMap.get(Servo.class, "gate b");

        intakeMotorA.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorB.setDirection(DcMotor.Direction.FORWARD);
        topShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterSpinupTimer = new ElapsedTime();
    }

    public void setIntakePower(double power) {
        intakeMotorA.setPower(power);
        intakeMotorB.setPower(power);
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
        double topVel = topShooterMotor.getVelocity();
        double bottomVel = bottomShooterMotor.getVelocity();
        return (Math.abs(topVel) > Math.abs(bottomVel)) ? topVel : bottomVel;
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
        double adjustedPower = calculateAdjustedShooterPowerForTargetRPM(targetRPM);
        setShooterPower(adjustedPower);
    }

    private double calculateAdjustedShooterPowerForTargetRPM(double targetRPM) {
        // Power off the motors if the target is zero.
        if (targetRPM == 0) {
//            Log.d(TAG, "calculateAdjustedShooterPowerForTargetRPM: Target RPM is 0, returning power 0.");
            return 0;
        }

        // Get the current average RPM of the shooter motors.
        double currentRPM = getShooterRPM();
        // Calculate the difference between the current RPM and the desired target RPM.
        double error = currentRPM - targetRPM;

        Log.d(TAG, String.format("calculateAdjustedShooterPowerForTargetRPM: Since=%.1f(s), Target=%.1f, Current=%.1f, Error=%.1f", shooterSpinupTimer.seconds(), targetRPM, currentRPM, error));

        // Check if the absolute error is within an acceptable tolerance.
        if (Math.abs(error) < SHOOTER_RPM_TOLERANCE) {
            // If the RPM is close enough to the target, no change is needed.
            Log.d(TAG, String.format("calculateAdjustedShooterPowerForTargetRPM: RPM is within tolerance. Maintaining current power: %.2f", shooterPower));
            return shooterPower;
        } else {
            // If the error is too large, a correction is needed.
            // Only apply a power correction after a certain time to avoid unstable oscillations.
            if (shooterPower > 0 && shooterSpinupTimer.seconds() > 1 && enablePowerAdjustment) {
//                Log.d(TAG, "calculateAdjustedShooterPowerForTargetRPM: RPM outside tolerance. Calculating correction.");
                // Apply a proportional correction to the current power.
                // If the shooter is too slow (error is negative), this increases the power.
                // If the shooter is too fast (error is positive), this decreases the power.
                double adjustedPower = shooterPower * (1.0 - error / targetRPM * powerAdjustementFactor);
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
    public boolean isShooterAtTargetRPM(double targetRPM) {
        double error = getShooterRPM() - targetRPM;
        return Math.abs(error) < SHOOTER_RPM_TOLERANCE;
    }

    /*** Predicts the motor power needed to achieve a target RPM, with voltage compensation.
     * This function is based on the linear model:
     *   y = a * x + b
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

        // Calculate 'y' (the Power * Voltage product) using the provided linear model.
        // double powerVoltageProduct = 2.1034 * x + 1.5139; // uses all recorded points
        // double powerVoltageProduct = 2.01014 * x + 1.75808; // uses points with x between 2.1 and 3.4
        double powerVoltageProduct = 2.09819 * x + 1.38837; // uses resting RPM after 7 seconds

        // Solve for the final power by dividing the result by the current voltage.
        double calculatedPower = powerVoltageProduct / currentVoltage;

        // Clamp the result to the valid motor power range [0.0, 1.0].
        return Range.clip(calculatedPower, 0.0, 1.0);
    }

    /**
     * Predicts the target shooter RPM based on the distance from the target.
     * The formula provided is:
     * y = 11.019x^4 - 204.17x^3 + 1396.3x^2 - 3998.5x + 6498.1
     * where 'x' is in tile units (1 tile = 24 inches) and 'y' is RPM.
     *
     * @param distanceInInches The distance from the target in inches.
     * @return The calculated target RPM for the shooter.
     */
    public double predictShooterRPMFromDistance(double distanceInInches) {
        // Convert the distance from inches to tile units, as required by the formula.
        double x = distanceInInches / 24.0;

        // Because of the polynomial fit, do not extrapolate outside the valid range.
        x = Range.clip(x, 2.1, 7); // tiles

        // Apply the 4th degree polynomial formula to calculate the target RPM.
        double rpm = 11.019 * Math.pow(x, 4)
                - 204.17 * Math.pow(x, 3)
                + 1396.3 * Math.pow(x, 2)
                - 3998.5 * x
                + 6498.1;

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

    // Gate Constants
    static public final double GATE_A_OPEN = 0.0;
    static public final double GATE_A_CLOSED = 0.20;
    static public final double GATE_B_OPEN = 0.35;
    static public final double GATE_B_CLOSED = 0.65;

    // Shooter Constants
    static private final double SHOOTER_TICKS_PER_REVOLUTION = 28;
    static public final double SHOOTER_RPM_LOW = 1000;
    static public final double SHOOTER_RPM_MAX = 5000;
    static public final double SHOOTER_RPM_TOLERANCE = 20;
    static public final double SHOOTER_RPM_INCREMENT = 40;

    private static final String TAG = "VEX::Actuators";
}
