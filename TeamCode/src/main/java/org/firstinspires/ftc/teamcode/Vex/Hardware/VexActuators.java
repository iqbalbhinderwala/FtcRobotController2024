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
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        topShooterMotor = hardwareMap.get(DcMotor.class, "top shooter");
        bottomShooterMotor = hardwareMap.get(DcMotor.class, "bottom shooter");

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

    public void shootSequence(double shooterPower) {
        // Turn on shooters
        setShooterPower(shooterPower);
        opMode.sleep(1000); // Give motors time to spin up

        // Run sequence up to 3 times, can be interrupted by releasing Y
        for (int i = 0; i < 3; i++) {
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // CLOSE GATE 2
            closeGateB(); // Closed
            opMode.sleep(250);
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // OPEN GATE 1
            openGateA(); // Open
            opMode.sleep(250);
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // CLOSE GATE 1
            closeGateA(); // Closed
            opMode.sleep(250);
            if (!opMode.opModeIsActive() || !opMode.gamepad1.y) break;
            // OPEN GATE 2
            openGateB(); // Open
            opMode.sleep(250);
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
    public final double GATE_A_CLOSED = 0.30;
    public final double GATE_B_OPEN = 0.35;
    public final double GATE_B_CLOSED = 0.65;
}
