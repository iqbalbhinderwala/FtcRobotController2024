package org.firstinspires.ftc.teamcode.John.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * John's gate / indexer ("indexer" in John.xml, Expansion Hub port 3): a REV Core Hex motor, 288
 * counts per output revolution. Each step turns it STEP_DEG (90) to the next absolute position, so
 * steps never add up errors; one step releases one ball. Call update() every loop.
 */
@Config("JohnIndexer")
public class Indexer {

    /** Encoder counts per output revolution (REV Core Hex). */
    public static double TICKS_PER_REV = 288;
    public static double STEP_DEG = 90;
    public static double POWER = 1.0;
    public static boolean REVERSED = true;

    private final DcMotorEx motor;
    private int step;

    public Indexer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "indexer");
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // positions count from here
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Moves {@code steps} positions (+ forward, - back). */
    public void step(int steps) {
        step += steps;
    }

    /** Applies the target for this loop. Call every loop. */
    public void update() {
        motor.setTargetPosition(targetTicks());
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(POWER);
    }

    public void stop() {
        motor.setPower(0);
    }

    private int targetTicks() {
        return (int) Math.round(step * STEP_DEG / 360.0 * TICKS_PER_REV);
    }

    public int stepCount() {
        return step;
    }

    public double degrees() {
        return motor.getCurrentPosition() / TICKS_PER_REV * 360.0;
    }

    public double targetDegrees() {
        return targetTicks() / TICKS_PER_REV * 360.0;
    }

    /** Still moving to the target (RUN_TO_POSITION). */
    public boolean busy() {
        return motor.isBusy();
    }

    public double amps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
}
