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
 *    Holding trim (TRIM): F is multiplied by a trim that follows the slow drift while the speed is
 *    steady (velocity control, no ball dip in the last TRIM_HOLDOFF_MS): John v2's gear belt turns
 *    easier as it warms, and after ~20 s on, F 23.31 held the flywheel 200-330 RPM fast (Auto r1069 /
 *    r1070: volley 2 never fired). Unlike the I term it is frozen during dips, so balls do not wind it
 *    up. It starts at 1 in each OpMode.
 *  - Stall guard: powered, under STALL_RPM and over STALL_AMPS for STALL_MS = the motor is not
 *    turning (Auto r1074: 10.3 A at full power, 0 RPM for 1 s, nothing blocking it; a healthy start is
 *    past 200 RPM in 0.2 s and drops from 9.4 to 6 A). Possibly a dead spot of the brushed motor, so
 *    first a short reverse kick (KICK_POWER for KICK_MS) and try again, up to STALL_KICKS times; then
 *    it stays off (stalled(): the Shooter skips the volley) until turned off and on again.
 *  - open-loop (USE_ENCODER false, or the encoder reads 0 while powered and ENCODER_FALLBACK is on):
 *    power = target / MAX_RPM, scaled by NOMINAL_VOLTAGE / battery voltage. Without encoder feedback
 *    the velocity control would run away to full power whatever the target.
 *
 * Speeds measured 2026-10-23 (shooting at the up CELL from the audience wall): about 2200 RPM from
 * C1, 2300 from B1, 2400 from A1 (see Shooter.RPM_DISTANCES_IN / RPM_VALUES).
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
     * John v2 (gear belt, r1057, 13.1 V, F 23.31): 100 below with I 1 never passed +100 in either
     * pass, ready in 2.2-2.6 s; 200 below overshot 143 in most trials.
     */
    public static double BOOST_EXIT_RPM = 100;
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
    /**
     * John v2: 25 (v1 10), so an overshoot or a slow drift comes back faster; with only F and P 10 a
     * spin-up overshoot of ~100 RPM took seconds to fade, and Shooter.FIRE_ABOVE_RPM now waits for it.
     */
    public static double P = 25.0;
    /** Small: F does the holding; I only corrects slow drift (r0046: 0 / 0.5 / 1 all fine, 3 once overshot). */
    /**
     * John v2: 0. The calibration (r1057, no balls) picked 1.0, but in Auto r1061 / r1063 the I term
     * built up over the ball dips and after shot 4-5 held the flywheel 125-133 RPM fast, so shot 6
     * waited 1.7-2.2 s for "at speed"; before a volley F alone held the target within ~15 RPM.
     */
    public static double I = 0.0;
    public static double D = 0.0;
    /**
     * Learned 2026-10-23 with [John] Flywheel Calibration (r0042): 20.45 / 20.17 / 20.86 at 12 V from
     * runs at 11.3 / 12.3 / 13.3 V, so the voltage compensation holds. The old 12 supplied only about
     * 60% of the holding power and left the rest to the I term (slow, overshooting).
     * Relearned 2026-11-01 (r0267, 11.5 V): 18.76. Holding 2300 RPM took 7.4 V (power x battery)
     * against 7.9-8.2 V on 10/23, so the flywheel itself got about 9% easier to turn (not the
     * battery); 20.5 held it 100-190 RPM fast, outside the "at speed" band, and stalled volleys.
     * John v2 (gear belt, 1:1 pulleys), r1057 at 13.15 V: RPM = 4767 x power - 1020 (0.3 / 0.4 / 0.5
     * -> 400 / 905 / 1354), so holding 2300 needs 0.696 -> F 23.31 at 12 V (+24%: the gear belt
     * takes more power to turn; full power still reaches ~3700 RPM).
     */
    public static double F = 23.31;
    /** Holding trim on F (see the class comment). */
    public static boolean TRIM = true;
    /**
     * Trim change per second per fractional speed error. 1.5: a few % error fades in under a second.
     * 0.5 was too slow (Auto r1083: after the last ball the flywheel sat 70-115 RPM fast for 5 s).
     */
    public static double TRIM_GAIN = 1.5;
    /** Frozen while more than this below the target (a ball dip) and TRIM_HOLDOFF_MS after it / a boost. */
    public static double TRIM_FREEZE_BELOW_RPM = 60;
    public static double TRIM_HOLDOFF_MS = 500;
    public static double TRIM_MIN = 0.8;
    public static double TRIM_MAX = 1.1;
    /** Stall guard (see the class comment). */
    public static boolean STALL_GUARD = true;
    public static double STALL_RPM = 100;
    public static double STALL_AMPS = 6.0;
    public static double STALL_MS = 300;
    public static double KICK_POWER = -0.4;
    public static double KICK_MS = 150;
    public static int STALL_KICKS = 2;

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
    private double trim = 1.0;
    private boolean stalled;
    private boolean kicking;
    private int kicks;
    private double stallAmps;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime kickTimer = new ElapsedTime();
    private final ElapsedTime sinceDisturbed = new ElapsedTime();
    private final ElapsedTime trimTimer = new ElapsedTime();

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
            stalled = false;
            kicking = false;
            kicks = 0;
            stallTimer.reset();
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

        // Stall guard: a reverse kick, or off after STALL_KICKS kicks
        if (on && (kicking || stalled)) {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boosting = false;
            if (kicking && kickTimer.milliseconds() >= KICK_MS) {
                kicking = false;
                stallTimer.reset();
                event = "flywheel kick done: trying again";
            }
            power = kicking ? KICK_POWER : 0.0;
            motor.setPower(power);
            rpm = motor.getVelocity() / ticksPerRev() * 60.0;
            if (kicking || stalled) {
                return;
            }
        }

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
            // Re-send only on a real change (the voltage compensation and the trim move in small steps)
            if (appliedPidf == null || appliedPidf[0] != P || appliedPidf[1] != I || appliedPidf[2] != D
                    || Math.abs(appliedPidf[3] - f) > 0.004 * Math.max(1e-6, Math.abs(f))) {
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
        updateTrim(closedLoop);
        // Not turning while drawing stall current (the current is read only while it is that slow)
        boolean slow = STALL_GUARD && on && power > 0.2 && Math.abs(rpm) < STALL_RPM;
        double amps = slow ? amps() : 0.0;
        if (slow && amps > STALL_AMPS) {
            stallAmps = Math.max(stallAmps, amps);
            if (stallTimer.milliseconds() >= STALL_MS) {
                if (kicks < STALL_KICKS) {
                    kicks++;
                    kicking = true;
                    kickTimer.reset();
                    event = String.format(java.util.Locale.US,
                            "flywheel STALLED (%.0f RPM at %.1f A): reverse kick %d", rpm, stallAmps, kicks);
                } else {
                    stalled = true;
                    event = String.format(java.util.Locale.US,
                            "flywheel STALLED (%.0f RPM at %.1f A) after %d kicks: off", rpm, stallAmps, kicks);
                }
                motor.setPower(0);
                stallAmps = 0.0;
                return;
            }
        } else {
            stallTimer.reset();
        }
        // An encoder that reads 0 while the motor turns (current not at stall) -> open-loop
        if (closedLoop && noEncoderSignal() && ENCODER_FALLBACK && !(STALL_GUARD && slow && amps > STALL_AMPS)) {
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

    /** F for the current battery voltage (F itself is for NOMINAL_VOLTAGE), times the holding trim. */
    public double effectiveF() {
        double f = VOLTAGE_COMPENSATION && voltage > 1.0 ? F * NOMINAL_VOLTAGE / voltage : F;
        return f * (TRIM ? trim : 1.0);
    }

    /** The holding trim on F (1 = F as calibrated). */
    public double trim() {
        return trim;
    }

    /** Follows the slow drift only while the speed is steady near the target; see the class comment. */
    private void updateTrim(boolean closedLoop) {
        double dt = Math.min(0.1, trimTimer.seconds());
        trimTimer.reset();
        boolean steady = TRIM && closedLoop && on && !boosting && targetRpm > 0
                && rpm >= targetRpm - TRIM_FREEZE_BELOW_RPM;
        if (!steady) {
            sinceDisturbed.reset();
            return;
        }
        if (sinceDisturbed.milliseconds() < TRIM_HOLDOFF_MS) {
            return;
        }
        double error = (rpm - targetRpm) / targetRpm;
        trim = Math.max(TRIM_MIN, Math.min(TRIM_MAX, trim - TRIM_GAIN * error * dt));
    }

    /** The stall guard turned it off (not turning at stall current, kicks did not help). */
    public boolean stalled() {
        return stalled;
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
        return within(TOLERANCE_RPM);
    }

    /** Within {@code toleranceRpm} of the target (encoder mode only). */
    public boolean within(double toleranceRpm) {
        return within(toleranceRpm, toleranceRpm);
    }

    /** At most {@code belowRpm} under and {@code aboveRpm} over the target (encoder mode only). */
    public boolean within(double belowRpm, double aboveRpm) {
        return closedLoop() && on && !boosting && rpm >= targetRpm - belowRpm && rpm <= targetRpm + aboveRpm;
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
