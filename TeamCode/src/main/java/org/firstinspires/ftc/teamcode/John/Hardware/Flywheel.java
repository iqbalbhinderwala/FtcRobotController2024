package org.firstinspires.ftc.teamcode.John.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * John's shooter flywheel ("shooter" in John.xml, Expansion Hub port 2): a bare goBILDA 5000 Series
 * motor (about 6000 RPM, encoder 28 counts per revolution). Call update() every loop.
 *
 * Modes:
 *  - encoder (USE_ENCODER, the default): the hub's velocity control (RUN_USING_ENCODER, setVelocity)
 *    with the PIDF below. Boost start (BOOST): while more than BOOST_ENTER_RPM below the target it
 *    runs at BOOST_POWER open-loop, then switches to the velocity control once within BOOST_EXIT_RPM.
 *    A cold start with the velocity control alone overshot by up to ~800 RPM and took seconds to
 *    settle (the I term winds up during the long spin-up). A deep dip from a ball (more than
 *    BOOST_ENTER_RPM) also gets a short boost, which recovers it fast.
 *  - open-loop (USE_ENCODER false, or the encoder reads 0 while powered and ENCODER_FALLBACK is on):
 *    power = target / MAX_RPM, scaled by NOMINAL_VOLTAGE / battery voltage. Without encoder feedback
 *    the velocity control would run away to full power whatever the target.
 *
 * Speeds measured 2026-10-23 (shooting at the up CELL from the audience wall): about 2200 RPM from
 * C1, 2300 from B1, 2400 from A1 (see Shooter.RPM_FROM_*).
 */
@Config("JohnFlywheel")
public class Flywheel {

    public static double DEFAULT_TARGET_RPM = 2300;
    public static double MAX_RPM = 6000;
    /** Encoder counts per motor revolution (goBILDA 5000 Series motor encoder). */
    public static double TICKS_PER_REV = 28;
    /** Motor turns per flywheel turn (1 if the flywheel is on the motor shaft). */
    public static double GEARING = 1.0;
    public static boolean REVERSED = false;
    /** Speed control with the encoder (connected and working since 2026-10-23); false = open-loop. */
    public static boolean USE_ENCODER = true;
    /** Switch to open-loop when the encoder reads 0 while powered (false: stay, to test an encoder). */
    public static boolean ENCODER_FALLBACK = true;
    /** Full-power spin-up, then the velocity control (avoids the cold-start overshoot). */
    public static boolean BOOST = true;
    public static double BOOST_POWER = 1.0;
    /**
     * Boost when more than this below the target (also after a deep dip from a ball). 150 vs 300
     * (r0051 vs r0050, 30 balls each): deepest dip 200 vs 243 RPM, back in the band within 0.19 vs
     * 0.48 s, never past the band afterwards.
     */
    public static double BOOST_ENTER_RPM = 150;
    /**
     * ... until within this of it. Calibrated 2026-10-23 (r0046, 13.3 V, F 20.5): with the hand-off 50-150
     * below and any I up to 1, cold starts peaked at most 100 above the target (inside the "at speed"
     * band) and were ready in about 1.7-1.8 s; later hand-offs are faster.
     */
    public static double BOOST_EXIT_RPM = 50;
    /** Battery voltage the open-loop power and F are given for. */
    public static double NOMINAL_VOLTAGE = 12.0;
    /**
     * Scale F by NOMINAL_VOLTAGE / battery voltage. The hub's output is a fraction of the battery
     * voltage, so the power that holds a speed goes up as the battery drains.
     */
    public static boolean VOLTAGE_COMPENSATION = true;
    /** "At speed" when within this of the target. */
    public static double TOLERANCE_RPM = 100;
    /**
     * The hub's velocity PIDF (F about 32767 / max ticks per second). F is for NOMINAL_VOLTAGE (see
     * VOLTAGE_COMPENSATION); [John] Flywheel Calibration learns it.
     */
    public static double P = 10.0;
    /** Small: F does the holding; I only corrects slow drift (r0046: 0 / 0.5 / 1 all fine, 3 once overshot). */
    public static double I = 0.5;
    public static double D = 0.0;
    /**
     * Learned 2026-10-23 with [John] Flywheel Calibration (r0042): 20.45 / 20.17 / 20.86 at 12 V from
     * runs at 11.3 / 12.3 / 13.3 V, so the voltage compensation holds. The old 12 supplied only about
     * 60% of the holding power and left the rest to the I term (slow, overshooting).
     */
    public static double F = 20.5;

    private final DcMotorEx motor;
    private final HardwareMap hardwareMap;
    private final ElapsedTime onFor = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private boolean on;
    private double targetRpm = DEFAULT_TARGET_RPM;
    private boolean boosting;
    private boolean encoderMissing;
    private double rpm;
    private double power;
    private double voltage = NOMINAL_VOLTAGE;
    private double[] appliedPidf;
    private String event;
    /** Calibration: raw open-loop power instead of the normal control, or null. */
    private Double manualPower;

    public Flywheel(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        // The real motor values (no SDK motor type matches the bare 5000 Series motor)
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setTicksPerRev(TICKS_PER_REV);
        type.setGearing(GEARING);
        type.setMaxRPM(MAX_RPM);
        motor.setMotorType(type);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast down, no hard braking
    }

    public void setOn(boolean on) {
        if (on && !this.on) {
            onFor.reset();
        }
        this.on = on;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(MAX_RPM, rpm));
    }

    /**
     * Calibration only: run at this raw power, open-loop (no boost, no velocity control, no voltage
     * scaling); null goes back to the normal control.
     */
    public void setManualPower(Double power) {
        manualPower = power;
    }

    /** Brake (true) or coast (false) when the power is zero; braking stops it quickly between trials. */
    public void setBrakeWhenOff(boolean brake) {
        motor.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Applies the mode and power for this loop and reads the speed. Call every loop. */
    public void update() {
        event = null;
        if (voltageTimer.seconds() > 0.25) {
            voltage = batteryVoltage(voltage);
            voltageTimer.reset();
        }
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (manualPower != null) {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boosting = false;
            power = manualPower;
            motor.setPower(power);
            rpm = motor.getVelocity() / ticksPerRev() * 60.0;
            return;
        }
        boolean closedLoop = closedLoop();

        // Boost far below the target, velocity control near it (hysteresis)
        double below = targetRpm - rpm;
        boolean wasBoosting = boosting;
        boosting = closedLoop && on && BOOST && (boosting ? below > BOOST_EXIT_RPM : below > BOOST_ENTER_RPM);
        if (boosting != wasBoosting) {
            event = boosting ? "flywheel boost" : "flywheel velocity control";
        }

        if (closedLoop && boosting) {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            power = BOOST_POWER;
            motor.setPower(power);
        } else if (closedLoop) {
            double f = effectiveF();
            // Re-send only on a real change (compensation follows the voltage in about 1% steps)
            if (appliedPidf == null || appliedPidf[0] != P || appliedPidf[1] != I || appliedPidf[2] != D
                    || Math.abs(appliedPidf[3] - f) > 0.01 * Math.max(1e-6, Math.abs(f))) {
                motor.setVelocityPIDFCoefficients(P, I, D, f);
                appliedPidf = new double[]{P, I, D, f};
            }
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // switching in starts the velocity control fresh
            if (on) {
                motor.setVelocity(targetRpm / 60.0 * ticksPerRev());
            } else {
                motor.setPower(0);
            }
            power = motor.getPower();
        } else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            power = on ? Math.max(0.0, Math.min(1.0, targetRpm / MAX_RPM * NOMINAL_VOLTAGE / voltage)) : 0.0;
            motor.setPower(power);
        }

        rpm = motor.getVelocity() / ticksPerRev() * 60.0;
        if (closedLoop && noEncoderSignal() && ENCODER_FALLBACK) {
            encoderMissing = true;
            event = "flywheel encoder reads 0: switched to open-loop";
        }
    }

    public void stop() {
        on = false;
        motor.setPower(0);
    }

    private void setMode(DcMotor.RunMode mode) {
        if (motor.getMode() != mode) {
            motor.setMode(mode);
        }
    }

    private double ticksPerRev() {
        return TICKS_PER_REV * GEARING;
    }

    private double batteryVoltage(double previous) {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 1.0) {
                return v;
            }
        }
        return previous;
    }

    /** F for the current battery voltage (F itself is for NOMINAL_VOLTAGE). */
    public double effectiveF() {
        return VOLTAGE_COMPENSATION && voltage > 1.0 ? F * NOMINAL_VOLTAGE / voltage : F;
    }

    /** Powered for over a second but the encoder reads 0. */
    public boolean noEncoderSignal() {
        return on && onFor.seconds() > 1.0 && Math.abs(rpm) < 1.0;
    }

    public boolean closedLoop() {
        return USE_ENCODER && !encoderMissing;
    }

    /** Within TOLERANCE_RPM of the target (encoder mode only). */
    public boolean atSpeed() {
        return closedLoop() && on && !boosting && Math.abs(rpm - targetRpm) <= TOLERANCE_RPM;
    }

    public boolean isOn() {
        return on;
    }

    public double onSeconds() {
        return on ? onFor.seconds() : 0.0;
    }

    public double targetRpm() {
        return targetRpm;
    }

    public double rpm() {
        return rpm;
    }

    public double power() {
        return power;
    }

    public double voltage() {
        return voltage;
    }

    public boolean boosting() {
        return boosting;
    }

    public boolean encoderMissing() {
        return encoderMissing;
    }

    public int ticks() {
        return motor.getCurrentPosition();
    }

    public double ticksPerSecond() {
        return motor.getVelocity();
    }

    public double amps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /** "boost", "encoder" or "open-loop". */
    public String mode() {
        return closedLoop() ? (boosting ? "boost" : "encoder") : "open-loop";
    }

    /** What changed in the last update() (boost on/off, encoder fallback), or null. */
    public String event() {
        return event;
    }
}
