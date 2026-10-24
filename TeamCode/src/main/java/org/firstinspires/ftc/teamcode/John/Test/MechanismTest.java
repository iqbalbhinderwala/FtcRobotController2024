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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.John.Hardware.Flywheel;
import org.firstinspires.ftc.teamcode.John.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

/**
 * Test of John's mechanisms (the robot does not drive): both intakes, the shooter flywheel and the
 * gate / indexer. Motor names from John.xml (Expansion Hub): intakeB 0, intakeA 1, shooter 2,
 * indexer 3. Flywheel and indexer settings live in John/Hardware (Dashboard: JohnFlywheel,
 * JohnIndexer).
 *
 * gamepad1:
 *   dpad right / left   both intakes together: in on/off / reverse on/off
 *   Y                   flywheel on/off (holds the target RPM; boost start, see Flywheel)
 *   dpad up / down      flywheel target + / - FLYWHEEL_STEP_RPM
 *   right / left bumper indexer + / - 90 deg
 *   X                   shoot SHOOT_COUNT balls (flywheel must be on); X again cancels
 *   A                   stop everything
 *
 * Both intakes always run together at INTAKE_POWER (full by default); INTAKE_A_REVERSED /
 * INTAKE_B_REVERSED flip which way "in" is for each (they sit at opposite ends).
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

    // ---- Flywheel target steps ----
    public static double FLYWHEEL_STEP_RPM = 100;

    // ---- Shoot sequence ----
    /** 4 balls, plus 2 extra steps in case a ball comes late. */
    public static int SHOOT_COUNT = 6;
    /** Pause between two balls, after the indexer step is done: 200-250 ms worked; 150 ms missed 1 in 4. */
    public static double SHOT_INTERVAL_MS = 250;
    /** Open-loop: time after switching the flywheel on before the first ball. */
    public static double SPIN_UP_MS = 1500;
    /**
     * Give up waiting for an indexer step after this long. Long enough for the indexer's own jam
     * recovery (back off, retry: about 0.75 s per try, see Indexer); a STUCK indexer ends the volley.
     */
    public static double STEP_TIMEOUT_MS = 4000;

    private enum IntakeState { OFF, IN, REVERSE }

    private enum ShootPhase { IDLE, SPIN_UP, STEP, INTERVAL }

    private static final String LOG_HEADER = "t_s,intakes,intake_a_amps,intake_b_amps,"
            + "flywheel_on,flywheel_mode,flywheel_target_rpm,flywheel_rpm,flywheel_ticks,flywheel_power,flywheel_amps,voltage,"
            + "indexer_step,indexer_target_deg,"
            + "indexer_deg,indexer_busy,indexer_amps,indexer_state,indexer_jams,event";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        DcMotorEx intakeA = hardwareMap.get(DcMotorEx.class, "intakeA");
        DcMotorEx intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        for (DcMotorEx intake : new DcMotorEx[]{intakeA, intakeB}) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        Flywheel flywheel = new Flywheel(hardwareMap);
        Indexer indexer = new Indexer(hardwareMap);

        CsvLog log = new CsvLog("john_mechanism_test", LOG_HEADER);
        telemetry.addLine("John Mechanism Test: the robot does not drive. Keep hands clear of the flywheel.");
        telemetry.addLine("dpad right / left intakes in / reverse, Y flywheel,");
        telemetry.addLine("dpad up/down flywheel speed, right/left bumper indexer +/-90 deg, X shoot, A stop all.");
        telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
        telemetry.update();
        waitForStart();
        gamepad1.resetEdgeDetection();

        IntakeState intakes = IntakeState.OFF;
        ElapsedTime clock = new ElapsedTime();
        ShootPhase shootPhase = ShootPhase.IDLE;
        int shotsDone = 0;
        ElapsedTime shootTimer = new ElapsedTime();

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
                flywheel.setOn(!flywheel.isOn());
                event = flywheel.isOn() ? "flywheel on" : "flywheel off";
            }
            if (faster) flywheel.setTargetRpm(flywheel.targetRpm() + FLYWHEEL_STEP_RPM);
            if (slower) flywheel.setTargetRpm(flywheel.targetRpm() - FLYWHEEL_STEP_RPM);
            if (idxPlus && shootPhase == ShootPhase.IDLE) {
                indexer.step(1);
                event = "indexer +" + Indexer.STEP_DEG;
            }
            if (idxMinus && shootPhase == ShootPhase.IDLE) {
                indexer.step(-1);
                event = "indexer -" + Indexer.STEP_DEG;
            }
            if (shoot) {
                if (shootPhase != ShootPhase.IDLE) {
                    shootPhase = ShootPhase.IDLE;
                    event = "shoot cancelled";
                } else if (flywheel.isOn()) {
                    shootPhase = ShootPhase.SPIN_UP;
                    shotsDone = 0;
                    event = "shoot " + SHOOT_COUNT;
                } else {
                    event = "shoot ignored: flywheel off";
                }
            }
            if (stopAll) {
                intakes = IntakeState.OFF;
                flywheel.setOn(false);
                shootPhase = ShootPhase.IDLE;
                event = "stop all";
            }
            if (in || reverse) event = "intakes " + intakes;

            // Intakes: fixed power
            intakeA.setDirection(INTAKE_A_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeB.setDirection(INTAKE_B_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            intakeA.setPower(intakePower(intakes));
            intakeB.setPower(intakePower(intakes));

            flywheel.update();
            if (flywheel.event() != null) {
                event = flywheel.event();
            }

            // Shoot sequence: one ball per indexer step, paced for the flywheel
            if (shootPhase != ShootPhase.IDLE && !flywheel.isOn()) {
                shootPhase = ShootPhase.IDLE;
                event = "shoot cancelled: flywheel off";
            }
            boolean flywheelReady = flywheel.closedLoop() ? flywheel.atSpeed()
                    : flywheel.onSeconds() * 1000.0 >= SPIN_UP_MS;
            switch (shootPhase) {
                case SPIN_UP:
                    if (flywheelReady) {
                        indexer.step(1);
                        shootPhase = ShootPhase.STEP;
                        shootTimer.reset();
                        event = "shot " + (shotsDone + 1);
                    }
                    break;
                case STEP:
                    if (indexer.stuck()) {
                        shootPhase = ShootPhase.IDLE;
                        event = "shoot stopped: indexer stuck, clear the ball";
                        break;
                    }
                    // busy() needs a moment to come on after a new target
                    if ((shootTimer.milliseconds() > 50 && !indexer.busy())
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
                    if (shootTimer.milliseconds() >= SHOT_INTERVAL_MS && (!flywheel.closedLoop() || flywheel.atSpeed())) {
                        indexer.step(1);
                        shootPhase = ShootPhase.STEP;
                        shootTimer.reset();
                        event = "shot " + (shotsDone + 1);
                    }
                    break;
                default:
                    break;
            }

            indexer.update();
            if (indexer.event() != null) {
                event = event == null ? indexer.event() : event + "; " + indexer.event();
            }

            double ampsA = intakeA.getCurrent(CurrentUnit.AMPS);
            double ampsB = intakeB.getCurrent(CurrentUnit.AMPS);
            double ampsFly = flywheel.amps();
            double ampsIdx = indexer.amps();
            int flywheelTicks = flywheel.ticks();

            telemetry.addData("Intakes (dpad right in / left reverse)", "%s, A %.1f A, B %.1f A", intakes, ampsA, ampsB);
            telemetry.addData("Flywheel (Y, dpad up/down)", "%s, target %.0f RPM, %s, %.1f A",
                    flywheel.isOn() ? "ON" : "off", flywheel.targetRpm(),
                    flywheel.closedLoop() ? String.format("actual %.0f RPM%s%s", flywheel.rpm(),
                            flywheel.atSpeed() ? " AT SPEED" : "", flywheel.boosting() ? " (BOOST)" : "")
                            : String.format("open-loop power %.2f at %.1f V", flywheel.power(), flywheel.voltage()),
                    ampsFly);
            telemetry.addData("Flywheel encoder", "%d ticks, %.0f ticks/s (%.0f RPM)  <- turn by hand: does it change?",
                    flywheelTicks, flywheel.ticksPerSecond(), flywheel.rpm());
            if (flywheel.encoderMissing()) {
                telemetry.addLine("Flywheel encoder reads 0: switched to open-loop. Connect the encoder cable?");
            } else if (flywheel.closedLoop() && flywheel.noEncoderSignal()) {
                telemetry.addLine("WARNING: encoder reads 0 and the fallback is off: the flywheel may run up to "
                        + "FULL POWER. Y or A to stop.");
            } else if (!Flywheel.USE_ENCODER && Math.abs(flywheel.rpm()) > 1.0) {
                telemetry.addData("Flywheel encoder", "%.0f RPM: it works, USE_ENCODER can be on", flywheel.rpm());
            }
            telemetry.addData("Indexer (RB + / LB -)", "step %d, target %.0f deg, at %.1f deg%s, %.1f A, %s, %d jams",
                    indexer.stepCount(), indexer.targetDegrees(), indexer.degrees(), indexer.busy() ? " (moving)" : "",
                    ampsIdx, indexer.state(), indexer.jams());
            if (indexer.stuck()) {
                telemetry.addLine("INDEXER STUCK: clear the ball, then RB / X to go on.");
            }
            telemetry.addData("Shoot (X)", shootPhase == ShootPhase.IDLE
                    ? String.format("ready: %d balls, %.0f ms apart", SHOOT_COUNT, SHOT_INTERVAL_MS)
                    : String.format("%s, %d / %d shot", shootPhase, shotsDone, SHOOT_COUNT));
            telemetry.addLine("A: stop everything");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("flywheelTargetRpm", flywheel.isOn() ? flywheel.targetRpm() : 0.0);
            packet.put("flywheelRpm", flywheel.rpm());
            packet.put("flywheelTicks", flywheelTicks);
            packet.put("indexerDeg", indexer.degrees());
            dashboard.sendTelemetryPacket(packet);

            log.row(clock.seconds(), intakes, ampsA, ampsB, flywheel.isOn() ? 1 : 0, flywheel.mode(),
                    flywheel.isOn() ? flywheel.targetRpm() : 0.0, flywheel.rpm(), flywheelTicks, flywheel.power(),
                    ampsFly, flywheel.voltage(), indexer.stepCount(), indexer.targetDegrees(),
                    indexer.degrees(), indexer.busy() ? 1 : 0, ampsIdx, indexer.state(), indexer.jams(), event);
        }
        intakeA.setPower(0);
        intakeB.setPower(0);
        flywheel.stop();
        indexer.stop();
        log.close();
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
