package org.firstinspires.ftc.teamcode.Vex.Hardware;

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
    private double shooterTargetPower = 0.0;
    private ElapsedTime shooterTimer = new ElapsedTime();



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

        shooterTimer = new ElapsedTime();
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void setShooterPower(double power) {
        shooterTargetPower = power;
        topShooterMotor.setPower(power);
        bottomShooterMotor.setPower(power);
    }

    public double getShooterTicksPerSecond() {
        return (topShooterMotor.getVelocity() + bottomShooterMotor.getVelocity()) / 2.0;
    }

    public double getShooterRPM() {
        return getShooterTicksPerSecond() / SHOOTER_TICKS_PER_REVOLUTION * 60.0;
    }

    public double getShooterTargetPower() {
        return shooterTargetPower;
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
        // Reset shooter timer on shooter starting up
        if (shooterTargetRPM == 0 && targetRPM > 0) {
            shooterTimer.reset();
        }

        shooterTargetRPM = Range.clip(targetRPM, 0, SHOOTER_RPM_MAX);

        if (shooterTargetRPM == 0) {
            setShooterPower(0); // stop the motors
            return;
        }

        double predictedPower = calculateShooterPowerForTargetRPM(shooterTargetRPM);
        setShooterPower(predictedPower);

        updateShooterPowerForTargetRPM();
    }

    /**
     * Checks if the shooter's current RPM is within the acceptable tolerance of the target RPM.
     * This method has NO side effects; it only returns a boolean state.
     * @return true if the shooter is at its target speed, false otherwise.
     */
    public boolean isShooterAtTargetRPM() {
        double error = getShooterRPM() - shooterTargetRPM;
        return Math.abs(error) < 20; // Use a constant for tolerance, e.g., SHOOTER_RPM_TOLERANCE
    }

    private boolean updateShooterPowerForTargetRPM() {
        // Get the current average RPM of the shooter motors.
        double currentRPM = getShooterRPM();
        // Calculate the difference between the current RPM and the desired target RPM.
        double error = currentRPM - shooterTargetRPM;

        // Check if the absolute error is within an acceptable tolerance (e.g., 20 RPM).
        if (Math.abs(error) < 20) {
            // If the RPM is close enough to the target, return true.
            return true;
        } else {
            // If the error is too large, a correction is needed.
            // Only apply a power correction every 0.1 seconds to avoid rapid, unstable oscillations.
            if (shooterTimer.seconds() > 0.1) {
                // Calculate the feed-forward power based on the target RPM and current voltage.
                // Apply a proportional correction to the predicted power.
                // If the shooter is too slow (error is negative), this increases the power.
                // If the shooter is too fast (error is positive), this decreases the power.
                double adjustedPower = shooterTargetPower * (1.0 - error / shooterTargetRPM);
                // Apply the newly calculated power to the motors.
                setShooterPower(adjustedPower);
            }
            // Return false because the shooter is not yet at the target speed.
            return false;
        }
    }

    /**
     * Calculates the shooter power based on the target RPM with voltage compensation.
     * This method is an alternative to using DcMotorEx.setVelocity() when you want to manage the power calculation manually.
     * It uses a linear model derived from experimental data to map RPM to power.
     *
     * @param targetRPM The desired rotations per minute for the shooter wheels.
     * @return The calculated motor power, a value between 0.0 and 1.0.
     */
    private double calculateShooterPowerForTargetRPM(double targetRPM) {
        if (targetRPM <= 0) {
            return 0.0;
        }

        // Get the current battery voltage.
        double voltage = getVoltage();

        // This scale factor is used to compensate for battery voltage drops.
        // It increases the target power when the voltage is below 12V
        // and decreases it when the voltage is above 12V.
        double scaleRPMFor12V = 1 + (12.0 - voltage) / 12.0; // same as: 2 - V/12
//        double scaleRPMFor12V = 12.0 / voltage; // simpler, slightly more aggressive

        // This formula converts the desired RPM into a motor power value.
        // It's based on a linear model (y = mx + b) where:
        // y = shooterTargetPower
        // m = 1 / SHOOTER_12V_RPM_TO_POWER_GAIN
        // x = targetShooterRPM * scaleRPMTo12V
        // b = -SHOOTER_12V_RPM_TO_POWER_OFFSET / SHOOTER_12V_RPM_TO_POWER_GAIN
        // The result is a power value that should achieve the target RPM at the current voltage.
        double calculatedPower = (scaleRPMFor12V * targetRPM - SHOOTER_12V_RPM_TO_POWER_OFFSET) / SHOOTER_12V_RPM_TO_POWER_GAIN;

        // Clamp the power to the valid range [0.0, 1.0] for the motor.
        return Range.clip(calculatedPower, 0.0, 1.0);
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
}
