package org.firstinspires.ftc.teamcode.Vex.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class VexActuators {
    private LinearOpMode opMode;

    // Declare OpMode members.
    private DcMotor intakeMotor = null;
    private DcMotor topShooterMotor = null;
    private DcMotor bottomShooterMotor = null;
    private Servo firstGateServo = null;
    private Servo secondGateServo = null;

    public VexActuators(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables.
        intakeMotor = hardwareMap.get(DcMotor.class, "m2");
        topShooterMotor = hardwareMap.get(DcMotor.class, "m0");
        bottomShooterMotor = hardwareMap.get(DcMotor.class, "m1");

        firstGateServo = hardwareMap.get(Servo.class, "gate a");
        secondGateServo = hardwareMap.get(Servo.class, "gate b");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        topShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{intakeMotor, topShooterMotor, bottomShooterMotor}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void setShooterPower(double power) {
        topShooterMotor.setPower(power);
        bottomShooterMotor.setPower(power);
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

    /**
     * Calculates the shooter power based on the distance to the alliance corner.
     * The formula is power(x in) = (4.48 / 24 * x + 55.055) / 100.
     * @return The calculated shooter power, a value between 0.0 and 1.0.
     */
    public double calculateDistanceBasedShooterPower(double distanceInInches) {
        double power = (4.48 / 24.0 * distanceInInches + 55.055) / 100.0;
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
    public final double GATE_A_OPEN = 0.0;
    public final double GATE_A_CLOSED = 0.20;
    public final double GATE_B_OPEN = 0.35;
    public final double GATE_B_CLOSED = 0.65;
}
