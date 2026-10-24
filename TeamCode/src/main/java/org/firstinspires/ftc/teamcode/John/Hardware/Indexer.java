package org.firstinspires.ftc.teamcode.John.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Locale;

/**
 * John's gate / indexer ("indexer" in John.xml, Expansion Hub port 3): a REV Core Hex motor, 288
 * counts per output revolution. Each step turns it STEP_DEG (90) to the next absolute position, so
 * steps never add up errors; one step releases one ball. Call update() every loop.
 *
 * Jams: a ball that arrives late gets caught by the indexer instead of rolling into place. In r0039
 * (2026-10-23) that stopped it dead at 3-4.7 A (about 0.2 A when turning freely) for 6-11 s while the
 * shoot sequence kept adding steps, up to 5 steps ahead; once the ball was freed by hand it spun
 * through all of them. Now:
 *  - JAM: still more than JAM_MIN_ERROR_DEG short of the target and less than JAM_PROGRESS_DEG of
 *    progress in JAM_MS (after JAM_GRACE_MS from a new target). Forward steps only: a normal step
 *    overshoots by ~20 deg and creeps back slowly, which must not count. On the r0039-r0051 logs
 *    this finds exactly the 4 jams of r0039 (2 cleared by themselves within 0.3 s) and nothing else.
 *  - BACK OFF: turn back BACKOFF_DEG (at most BACKOFF_MS), then wait SETTLE_MS so the ball can roll
 *    into place, then try the step again. After MAX_RETRIES jams in a row on the same step it gives
 *    up (STUCK): the motor is unpowered until the next step() call, so the ball can be freed by hand.
 *  - Steps do not pile up: the target is never more than MAX_AHEAD_STEPS ahead of where the indexer
 *    really is, so after a jam it only goes to the next position.
 */
@Config("JohnIndexer")
public class Indexer {

    /** Encoder counts per output revolution (REV Core Hex). */
    public static double TICKS_PER_REV = 288;
    public static double STEP_DEG = 90;
    public static double POWER = 1.0;
    public static boolean REVERSED = true;

    // ---- Jam handling ----
    public static boolean JAM_DETECTION = true;
    public static double JAM_MIN_ERROR_DEG = 30;
    public static double JAM_PROGRESS_DEG = 5;
    public static double JAM_MS = 250;
    public static double JAM_GRACE_MS = 150;
    public static double BACKOFF_DEG = 45;
    public static double BACKOFF_POWER = 0.6;
    public static double BACKOFF_MS = 300;
    public static double SETTLE_MS = 200;
    public static int MAX_RETRIES = 3;
    /** The target may be at most this many steps ahead of the indexer's real position. */
    public static int MAX_AHEAD_STEPS = 1;

    public enum State { RUNNING, BACKING_OFF, SETTLING, STUCK }

    private final DcMotorEx motor;
    private final ElapsedTime timer = new ElapsedTime();
    private int step;
    private State state = State.RUNNING;
    private double degrees;
    // Progress check
    private int checkedTarget = Integer.MIN_VALUE;
    private double targetSetMs;
    private double checkDeg;
    private double checkMs = -1;
    // Recovery
    private int retries;
    private int jams;
    private double backoffTargetDeg;
    private double phaseStartMs;
    private String event;

    public Indexer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "indexer");
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // positions count from here
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Moves {@code steps} positions (+ forward, - back), but never more than MAX_AHEAD_STEPS past
     * where the indexer really is (or as far behind). After STUCK it starts again from there.
     */
    public void step(int steps) {
        double at = degrees / STEP_DEG;
        if (state == State.STUCK) {
            state = State.RUNNING;
            step = (int) Math.round(at);
            retries = 0;
        }
        int max = (int) Math.round(at) + Math.max(1, MAX_AHEAD_STEPS);
        int min = (int) Math.round(at) - Math.max(1, MAX_AHEAD_STEPS);
        step = Math.max(min, Math.min(max, step + steps));
    }

    /** Reads the position, handles jams and drives toward the target. Call every loop. */
    public void update() {
        event = null;
        degrees = motor.getCurrentPosition() / TICKS_PER_REV * 360.0;
        double now = timer.milliseconds();
        int target = targetTicks();
        if (target != checkedTarget) {
            checkedTarget = target;
            targetSetMs = now;
            checkMs = -1;
        }
        double error = targetDegrees() - degrees;

        switch (state) {
            case RUNNING:
                if (Math.abs(error) < JAM_PROGRESS_DEG) {
                    retries = 0;   // got there
                }
                if (JAM_DETECTION && error > JAM_MIN_ERROR_DEG && now - targetSetMs >= JAM_GRACE_MS) {
                    if (checkMs < 0 || degrees - checkDeg >= JAM_PROGRESS_DEG) {
                        checkDeg = degrees;
                        checkMs = now;
                    } else if (now - checkMs >= JAM_MS) {
                        jams++;
                        retries++;
                        checkMs = -1;
                        if (retries > MAX_RETRIES) {
                            state = State.STUCK;
                            event = String.format(Locale.US, "indexer STUCK at %.0f deg (target %.0f) after %d tries: "
                                    + "clear the ball", degrees, targetDegrees(), MAX_RETRIES);
                        } else {
                            state = State.BACKING_OFF;
                            backoffTargetDeg = degrees - BACKOFF_DEG;
                            phaseStartMs = now;
                            event = String.format(Locale.US, "indexer jam at %.0f deg (target %.0f, %.1f A): backing off, try %d",
                                    degrees, targetDegrees(), amps(), retries);
                        }
                    }
                } else {
                    checkMs = -1;
                }
                drive(target, POWER);
                break;
            case BACKING_OFF:
                drive(ticks(backoffTargetDeg), BACKOFF_POWER);
                if (degrees <= backoffTargetDeg + JAM_PROGRESS_DEG || now - phaseStartMs >= BACKOFF_MS) {
                    state = State.SETTLING;
                    phaseStartMs = now;
                }
                break;
            case SETTLING:
                drive(ticks(backoffTargetDeg), BACKOFF_POWER);
                if (now - phaseStartMs >= SETTLE_MS) {
                    state = State.RUNNING;
                    targetSetMs = now;   // a new grace period for the retry
                    checkMs = -1;
                    event = "indexer retry";
                }
                break;
            case STUCK:
                drive(ticks(degrees), 0.0);
                break;
            default:
                break;
        }
    }

    private void drive(int targetTicks, double power) {
        motor.setTargetPosition(targetTicks);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }

    private int targetTicks() {
        return ticks(step * STEP_DEG);
    }

    private static int ticks(double deg) {
        return (int) Math.round(deg / 360.0 * TICKS_PER_REV);
    }

    public int stepCount() {
        return step;
    }

    /** Position read in the last update(). */
    public double degrees() {
        return degrees;
    }

    public double targetDegrees() {
        return targetTicks() / TICKS_PER_REV * 360.0;
    }

    /** Still moving to the target, or recovering from a jam (false when STUCK). */
    public boolean busy() {
        return state == State.BACKING_OFF || state == State.SETTLING || (state == State.RUNNING && motor.isBusy());
    }

    /** Gave up on a jam: clear the ball, then step() again. */
    public boolean stuck() {
        return state == State.STUCK;
    }

    public State state() {
        return state;
    }

    /** Jams detected since the start. */
    public int jams() {
        return jams;
    }

    /** What happened in the last update() (jam, retry, stuck), or null. */
    public String event() {
        return event;
    }

    public double amps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
}
