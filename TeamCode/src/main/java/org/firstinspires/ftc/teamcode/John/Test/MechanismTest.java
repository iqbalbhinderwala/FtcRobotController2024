package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

/**
 * Test of John's mechanisms (the robot does not drive): both intakes, the shooter flywheel and the
 * gate / indexer. Motor names from John.xml (Expansion Hub): intakeB 0, intakeA 1, shooter 2,
 * indexer 3.
 *
 * gamepad1:
 *   dpad right / left   both intakes together: in on/off / reverse on/off
 *   Y                   flywheel on/off (holds FLYWHEEL_TARGET_RPM)
 *   dpad up / down      flywheel target + / - FLYWHEEL_STEP_RPM
 *   right / left bumper indexer + / - 90 deg
 *   X                   shoot SHOOT_COUNT balls (flywheel must be on); X again cancels
 *   A                   stop everything
 *
 * Both intakes always run together at INTAKE_POWER (full by default); INTAKE_A_REVERSED /
 * INTAKE_B_REVERSED flip which way "in" is for each (they sit at opposite ends).
 * Flywheel, two modes:
 *  - open-loop (FLYWHEEL_USE_ENCODER false, for a flywheel without encoder): power =
 *    target RPM / FLYWHEEL_MAX_RPM, scaled by NOMINAL_VOLTAGE / battery voltage so the speed does not
 *    sag as the battery drains. It does not recover speed after a shot.
 *  - encoder (FLYWHEEL_USE_ENCODER true, the default): the hub's velocity control (RUN_USING_ENCODER,
 *    setVelocity) with the PIDF below, 28 counts per revolution and FLYWHEEL_GEARING. If the encoder
 *    reads 0 while powered it switches to open-loop for the rest of the run: without feedback the
 *    velocity control would wind up to full power whatever the target. FLYWHEEL_ENCODER_FALLBACK
 *    false stays in encoder mode anyway, to test an encoder (careful: no feedback = full power).
 * Indexer: REV Core Hex, 288 counts per output revolution; each press moves to the next multiple of
 * 90 deg (absolute targets, so steps do not add up errors).
 * Shoot sequence (X): one ball per 90 deg indexer step. Before the first ball it waits for the
 * flywheel to spin up (open-loop: SPIN_UP_MS after it was switched on; encoder: at speed); after each
 * step it waits for the step to finish, then SHOT_INTERVAL_MS (and, with the encoder, for the
 * flywheel to be back at speed) before the next ball. Each release is logged ("shot n").
 *
 * Logged to /sdcard/FIRST/john_logs/r<run>_john_mechanism_test_<time>.csv.
 */
@Config("JohnMechanismTest")
@TeleOp(name = "[John] Mechanism Test", group = "JohnTest")
public class MechanismTest extends LinearOpMode {

    // ---- Intakes ----
    public static double INTAKE_POWER = 1.0;
    public static boolean INTAKE_A_REVERSED = false;
    public static boolean INTAKE_B_REVERSED = false;

    // ---- Flywheel ----
    /** Measured 2026-10-23: about 2200 RPM from the C1 tile, 2300 from B1, 2400 from A1. */
    public static double FLYWHEEL_TARGET_RPM = 2300;
    public static double FLYWHEEL_STEP_RPM = 100;
    public static double FLYWHEEL_MAX_RPM = 6000;
    /** Encoder counts per motor revolution (goBILDA 5000 Series motor encoder). */
    public static double FLYWHEEL_TICKS_PER_REV = 28;
    /** Motor turns per flywheel turn (1 if the flywheel is on the motor shaft). */
    public static double FLYWHEEL_GEARING = 1.0;
    public static boolean FLYWHEEL_REVERSED = false;
    /** Speed control with the encoder (connected and working since 2026-10-23); false = open-loop power. */
    public static boolean FLYWHEEL_USE_ENCODER = true;
    /**
     * Switch to open-loop when the encoder reads 0 while powered. False stays in encoder mode, to test
     * an encoder; with no feedback the velocity control then runs up to full power.
     */
    public static boolean FLYWHEEL_ENCODER_FALLBACK = true;
    /** Battery voltage the open-loop power is scaled for. */
    public static double NOMINAL_VOLTAGE = 12.0;
    /** "At speed" when within this of the target. */
    public static double FLYWHEEL_TOLERANCE_RPM = 100;
    /** The hub's velocity PIDF (F about 32767 / max ticks per second). */
    public static double FLYWHEEL_P = 10.0;
    public static double FLYWHEEL_I = 3.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 12.0;

    // ---- Indexer ----
    /** Encoder counts per output revolution (REV Core Hex). */
    public static double INDEXER_TICKS_PER_REV = 288;
    public static double INDEXER_STEP_DEG = 90;
    public static double INDEXER_POWER = 1.0;
    public static boolean INDEXER_REVERSED = true;

    // ---- Shoot sequence ----
    /** 4 balls, plus 2 extra steps in case a ball comes late. */
    public static int SHOOT_COUNT = 6;
    /** Pause between two balls, after the indexer step is done (flywheel recovery). */
    /** 200-250 ms worked; at 150 ms 1 ball in 4 was missed (2026-10-23). */
    public static double SHOT_INTERVAL_MS = 250;
    /** Open-loop: time after switching the flywheel on before the first ball. */
    public static double SPIN_UP_MS = 1500;
    /** Give up waiting for an indexer step after this long. */
    public static double STEP_TIMEOUT_MS = 1000;

    private enum IntakeState { OFF, IN, REVERSE }

    private enum ShootPhase { IDLE, SPIN_UP, STEP, INTERVAL }

    private static final String LOG_HEADER = "t_s,intakes,intake_a_amps,intake_b_amps,"
            + "flywheel_on,flywheel_mode,flywheel_target_rpm,flywheel_rpm,flywheel_ticks,flywheel_power,flywheel_amps,voltage,"
            + "indexer_step,indexer_target_deg,"
            + "indexer_deg,indexer_busy,indexer_amps,event";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        DcMotorEx intakeA = hardwareMap.get(DcMotorEx.class, "intakeA");
        DcMotorEx intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotorEx indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        for (DcMotorEx intake : new DcMotorEx[]{intakeA, intakeB}) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Flywheel: the real motor values (no SDK motor type matches the bare 5000 Series motor)
        MotorConfigurationType flywheelType = flywheel.getMotorType().clone();
        flywheelType.setTicksPerRev(FLYWHEEL_TICKS_PER_REV);
        flywheelType.setGearing(FLYWHEEL_GEARING);
        flywheelType.setMaxRPM(FLYWHEEL_MAX_RPM);
        flywheel.setMotorType(flywheelType);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // coast down, no hard braking

        // Indexer: position control from where it is now
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setTargetPosition(0);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CsvLog log = new CsvLog("john_mechanism_test", LOG_HEADER);
        telemetry.addLine("John Mechanism Test: the robot does not drive. Keep hands clear of the flywheel.");
        telemetry.addLine("dpad right / left intakes in / reverse, Y flywheel,");
        telemetry.addLine("dpad up/down flywheel speed, right/left bumper indexer +/-90 deg, X shoot, A stop all.");
        telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
        telemetry.update();
        waitForStart();
        gamepad1.resetEdgeDetection();

        IntakeState intakes = IntakeState.OFF;
        boolean flywheelOn = false;
        int indexerStep = 0;
        ElapsedTime clock = new ElapsedTime();
        ElapsedTime flywheelOnFor = new ElapsedTime();
        double[] appliedPidf = null;
        boolean encoderMissing = false;
        ShootPhase shootPhase = ShootPhase.IDLE;
        int shotsDone = 0;
        ElapsedTime shootTimer = new ElapsedTime();
        double voltage = NOMINAL_VOLTAGE;
        ElapsedTime voltageTimer = new ElapsedTime();

        while (opModeIsActive()) {
            String event = null;
            // Read every button each loop (each read uses up the press)
            boolean in = gamepad1.dpadRightWasPressed(), reverse = gamepad1.dpadLeftWasPressed();
            boolean fly = gamepad1.yWasPressed();
            boolean faster = gamepad1.dpadUpWasPressed(), slower = gamepad1.dpadDownWasPressed();
            boolean idxPlus = gamepad1.rightBumperWasPressed(), idxMinus = gamepad1.leftBumperWasPressed();
            boolean stopAll = gamepad1.aWasPressed();
            boolean shoot = gamepad1.xWasPressed();

            if (in) intakes = intakes == IntakeState.IN ? IntakeState.OFF : IntakeState.IN;
            if (reverse) intakes = intakes == IntakeState.REVERSE ? IntakeState.OFF : IntakeState.REVERSE;
            if (fly) {
                flywheelOn = !flywheelOn;
                flywheelOnFor.reset();
                event = flywheelOn ? "flywheel on" : "flywheel off";
            }
            if (faster) FLYWHEEL_TARGET_RPM = Math.min(FLYWHEEL_MAX_RPM, FLYWHEEL_TARGET_RPM + FLYWHEEL_STEP_RPM);
            if (slower) FLYWHEEL_TARGET_RPM = Math.max(0, FLYWHEEL_TARGET_RPM - FLYWHEEL_STEP_RPM);
            if (idxPlus && shootPhase == ShootPhase.IDLE) {
                indexerStep++;
                event = "indexer +" + INDEXER_STEP_DEG;
            }
            if (idxMinus && shootPhase == ShootPhase.IDLE) {
                indexerStep--;
                event = "indexer -" + INDEXER_STEP_DEG;
            }
            if (shoot) {
                if (shootPhase != ShootPhase.IDLE) {
                    shootPhase = ShootPhase.IDLE;
                    event = "shoot cancelled";
                } else if (flywheelOn) {
                    shootPhase = ShootPhase.SPIN_UP;
                    shotsDone = 0;
                    event = "shoot " + SHOOT_COUNT;
                } else {
                    event = "shoot ignored: flywheel off";
                }
            }
            if (stopAll) {
                intakes = IntakeState.OFF;
                flywheelOn = false;
                shootPhase = ShootPhase.IDLE;
                event = "stop all";
            }
            if (in || reverse) event = "intakes " + intakes;

            // Intakes: fixed power
            intakeA.setDirection(INTAKE_A_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeB.setDirection(INTAKE_B_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeA.setPower(intakePower(intakes));
            intakeB.setPower(intakePower(intakes));

            // Flywheel: open-loop power, or speed control with the encoder
            if (voltageTimer.seconds() > 0.25) {
                voltage = batteryVoltage(voltage);
                voltageTimer.reset();
            }
            flywheel.setDirection(FLYWHEEL_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            double ticksPerFlywheelRev = FLYWHEEL_TICKS_PER_REV * FLYWHEEL_GEARING;
            boolean closedLoop = FLYWHEEL_USE_ENCODER && !encoderMissing;
            double flywheelPower = 0.0;
            if (closedLoop) {
                double[] pidf = {FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F};
                if (appliedPidf == null || !java.util.Arrays.equals(pidf, appliedPidf)) {
                    flywheel.setVelocityPIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]);
                    appliedPidf = pidf;
                }
                if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (flywheelOn) {
                    flywheel.setVelocity(FLYWHEEL_TARGET_RPM / 60.0 * ticksPerFlywheelRev);
                } else {
                    flywheel.setPower(0);
                }
                flywheelPower = flywheel.getPower();
            } else {
                if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                flywheelPower = flywheelOn
                        ? Math.max(0.0, Math.min(1.0, FLYWHEEL_TARGET_RPM / FLYWHEEL_MAX_RPM * NOMINAL_VOLTAGE / voltage))
                        : 0.0;
                flywheel.setPower(flywheelPower);
            }
            double flywheelRpm = flywheel.getVelocity() / ticksPerFlywheelRev * 60.0;
            boolean atSpeed = closedLoop && flywheelOn && Math.abs(flywheelRpm - FLYWHEEL_TARGET_RPM) <= FLYWHEEL_TOLERANCE_RPM;
            boolean noEncoder = flywheelOn && flywheelOnFor.seconds() > 1.0 && Math.abs(flywheelRpm) < 1.0;
            if (closedLoop && noEncoder && FLYWHEEL_ENCODER_FALLBACK) {
                encoderMissing = true;   // no feedback: the velocity control would run away to full power
                event = "flywheel encoder reads 0: switched to open-loop";
            }

            // Shoot sequence: one ball per indexer step, paced for the flywheel
            if (shootPhase != ShootPhase.IDLE && !flywheelOn) {
                shootPhase = ShootPhase.IDLE;
                event = "shoot cancelled: flywheel off";
            }
            boolean flywheelReady = closedLoop ? atSpeed : flywheelOnFor.milliseconds() >= SPIN_UP_MS;
            switch (shootPhase) {
                case SPIN_UP:
                    if (flywheelReady) {
                        indexerStep++;
                        shootPhase = ShootPhase.STEP;
                        shootTimer.reset();
                        event = "shot " + (shotsDone + 1);
                    }
                    break;
                case STEP:
                    // isBusy() needs a moment to come on after a new target
                    if ((shootTimer.milliseconds() > 50 && !indexer.isBusy())
                            || shootTimer.milliseconds() > STEP_TIMEOUT_MS) {
                        shotsDone++;
                        if (shotsDone >= SHOOT_COUNT) {
                            shootPhase = ShootPhase.IDLE;
                            event = "shoot done";
                        } else {
                            shootPhase = ShootPhase.INTERVAL;
                            shootTimer.reset();
                        }
                    }
                    break;
                case INTERVAL:
                    if (shootTimer.milliseconds() >= SHOT_INTERVAL_MS && (!closedLoop || atSpeed)) {
                        indexerStep++;
                        shootPhase = ShootPhase.STEP;
                        shootTimer.reset();
                        event = "shot " + (shotsDone + 1);
                    }
                    break;
                default:
                    break;
            }

            // Indexer: absolute 90 deg steps
            indexer.setDirection(INDEXER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            int target = (int) Math.round(indexerStep * INDEXER_STEP_DEG / 360.0 * INDEXER_TICKS_PER_REV);
            indexer.setTargetPosition(target);
            indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            indexer.setPower(INDEXER_POWER);
            double indexerDeg = indexer.getCurrentPosition() / INDEXER_TICKS_PER_REV * 360.0;
            double indexerTargetDeg = target / INDEXER_TICKS_PER_REV * 360.0;

            double ampsA = intakeA.getCurrent(CurrentUnit.AMPS);
            double ampsB = intakeB.getCurrent(CurrentUnit.AMPS);
            double ampsFly = flywheel.getCurrent(CurrentUnit.AMPS);
            double ampsIdx = indexer.getCurrent(CurrentUnit.AMPS);

            telemetry.addData("Intakes (dpad right in / left reverse)", "%s, A %.1f A, B %.1f A", intakes, ampsA, ampsB);
            telemetry.addData("Flywheel (Y, dpad up/down)", "%s, target %.0f RPM, %s, %.1f A",
                    flywheelOn ? "ON" : "off", FLYWHEEL_TARGET_RPM,
                    closedLoop ? String.format("actual %.0f RPM%s", flywheelRpm, atSpeed ? " AT SPEED" : "")
                            : String.format("open-loop power %.2f at %.1f V", flywheelPower, voltage), ampsFly);
            int flywheelTicks = flywheel.getCurrentPosition();
            telemetry.addData("Flywheel encoder", "%d ticks, %.0f ticks/s (%.0f RPM)  <- turn by hand: does it change?",
                    flywheelTicks, flywheel.getVelocity(), flywheelRpm);
            if (encoderMissing) {
                telemetry.addLine("Flywheel encoder reads 0: switched to open-loop. Connect the encoder cable?");
            } else if (closedLoop && noEncoder) {
                telemetry.addLine("WARNING: encoder reads 0 and the fallback is off: the flywheel may run up to "
                        + "FULL POWER. Y or A to stop.");
            } else if (!FLYWHEEL_USE_ENCODER && Math.abs(flywheelRpm) > 1.0) {
                telemetry.addData("Flywheel encoder", "%.0f RPM: it works, FLYWHEEL_USE_ENCODER can be on", flywheelRpm);
            }
            telemetry.addData("Indexer (RB + / LB -)", "step %d, target %.0f deg, at %.1f deg%s, %.1f A",
                    indexerStep, indexerTargetDeg, indexerDeg, indexer.isBusy() ? " (moving)" : "", ampsIdx);
            telemetry.addData("Shoot (X)", shootPhase == ShootPhase.IDLE
                    ? String.format("ready: %d balls, %.0f ms apart", SHOOT_COUNT, SHOT_INTERVAL_MS)
                    : String.format("%s, %d / %d shot", shootPhase, shotsDone, SHOOT_COUNT));
            telemetry.addLine("A: stop everything");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("flywheelTargetRpm", flywheelOn ? FLYWHEEL_TARGET_RPM : 0.0);
            packet.put("flywheelRpm", flywheelRpm);
            packet.put("flywheelTicks", flywheelTicks);
            packet.put("indexerDeg", indexerDeg);
            dashboard.sendTelemetryPacket(packet);

            log.row(clock.seconds(), intakes, ampsA, ampsB, flywheelOn ? 1 : 0, closedLoop ? "encoder" : "open-loop",
                    flywheelOn ? FLYWHEEL_TARGET_RPM : 0.0, flywheelRpm, flywheelTicks, flywheelPower, ampsFly, voltage,
                    indexerStep, indexerTargetDeg,
                    indexerDeg, indexer.isBusy() ? 1 : 0, ampsIdx, event);
        }
        intakeA.setPower(0);
        intakeB.setPower(0);
        flywheel.setPower(0);
        indexer.setPower(0);
        log.close();
    }

    /** Battery voltage (the Expansion / Control Hub's input), or the previous value if unreadable. */
    private double batteryVoltage(double previous) {
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 1.0) {
                return v;
            }
        }
        return previous;
    }

    private static double intakePower(IntakeState state) {
        switch (state) {
            case IN:
                return INTAKE_POWER;
            case REVERSE:
                return -INTAKE_POWER;
            default:
                return 0.0;
        }
    }
}
