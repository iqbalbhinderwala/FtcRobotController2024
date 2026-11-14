package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.John.Hardware.Defender;
import org.firstinspires.ftc.teamcode.John.Hardware.FallbackDrive;
import org.firstinspires.ftc.teamcode.John.Hardware.Flywheel;
import org.firstinspires.ftc.teamcode.John.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.John.Hardware.Intake;
import org.firstinspires.ftc.teamcode.John.Hardware.StatusLights;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;
import org.firstinspires.ftc.teamcode.John.Vision.HiveCamera;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Guided check of every device after a rebuild: one at a time, what should move and which way, with
 * the code's current settings (motor names, directions, REVERSED flags). Answer each step; the last
 * page lists what to change. Names are as the robot sees them: the SHOOTER side, INTAKE A on its
 * left, INTAKE B on its right, the CAMERA wall opposite the shooter; Pedro's front (+X) is the
 * intake B end, its left (+Y) the shooter side.
 *
 * gamepad1:
 *   right trigger (hold)  run the device of this step (released: it stops)
 *   A  looks right   B  wrong   X  skip      dpad left / right  previous / next step
 * Drive motor steps: John on a stand, wheels off the floor. Pinpoint / IMU steps: on the floor.
 *
 * Logged to /sdcard/FIRST/john_v2_logs/r<run>_john_hardware_check_<time>.csv (one row per answer).
 */
@Config("JohnHardwareCheck")
@TeleOp(name = "[John] Hardware Check", group = "JohnTest")
public class HardwareCheck extends LinearOpMode {

    public static double DRIVE_POWER = 0.3;
    public static double INTAKE_POWER = 0.5;
    public static double FLYWHEEL_POWER = 0.3;

    private enum Answer { NONE, OK, WRONG, SKIPPED }

    private static final class Step {
        final String name, expect, ifWrong;
        Answer answer = Answer.NONE;

        Step(String name, String expect, String ifWrong) {
            this.name = name;
            this.expect = expect;
            this.ifWrong = ifWrong;
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        String[] driveNames = {Constants.drivetrainConfig.frontLeftName.get(), Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.backLeftName.get(), Constants.drivetrainConfig.backRightName.get()};
        DcMotorSimple.Direction[] driveDirs = {Constants.drivetrainConfig.frontLeftDirection.get(),
                Constants.drivetrainConfig.frontRightDirection.get(), Constants.drivetrainConfig.backLeftDirection.get(),
                Constants.drivetrainConfig.backRightDirection.get()};
        String[] wheel = {"front-left (intake B end, shooter side)", "front-right (intake B end, camera side)",
                "back-left (intake A end, shooter side)", "back-right (intake A end, camera side)"};
        String[] dirField = {"frontLeftDirection", "frontRightDirection", "backLeftDirection", "backRightDirection"};
        DcMotorEx[] drive = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            drive[i] = hardwareMap.get(DcMotorEx.class, driveNames[i]);
            drive[i].setDirection(driveDirs[i]);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        DcMotorEx intakeA = hardwareMap.get(DcMotorEx.class, "intakeA");
        DcMotorEx intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        Intake intake = new Intake(hardwareMap);
        Indexer indexer = new Indexer(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Defender defender = new Defender(hardwareMap);
        StatusLights lights = new StatusLights(hardwareMap);
        FallbackDrive imu = new FallbackDrive(hardwareMap);
        HiveCamera camera = new HiveCamera(hardwareMap);
        Follower follower = Constants.create(hardwareMap);
        CsvLog log = new CsvLog("john_hardware_check", "t_s,step,answer,reading");

        List<Step> steps = new ArrayList<>();
        steps.add(new Step("Setup", "John on a stand, wheels off the floor, for the 4 wheel steps. A to start.", ""));
        for (int i = 0; i < 4; i++) {
            steps.add(new Step("Wheel " + driveNames[i],
                    "Hold RT: ONLY the " + wheel[i] + " wheel turns, rolling as if John drives toward intake B. Its own encoder moves (+).",
                    "Other wheel turned: fix the motor names / ports in John.xml. Right wheel, wrong way: flip "
                            + dirField[i] + " in Pedro/Constants.java."));
        }
        steps.add(new Step("Pinpoint forward", "On the floor. RT resets. Push John 12\" toward intake B: X goes up about +12, Y stays near 0.",
                "X went down: flip xPodDirection in Constants. Y moved instead: the pod cables are swapped on the Pinpoint."));
        steps.add(new Step("Pinpoint sideways", "RT resets. Push John 12\" toward the shooter side: Y goes up about +12, X stays near 0.",
                "Y went down: flip yPodDirection in Constants."));
        steps.add(new Step("Pinpoint turn", "RT resets. Turn John 90 deg counter-clockwise in place: heading +90, X and Y stay within about 1\".",
                "Heading went down: both pod directions or the Pinpoint orientation. X / Y swing a lot: pod offsets (Odometry Test)."));
        steps.add(new Step("Hub IMU", "Turn John counter-clockwise: the IMU heading goes up.",
                "It goes down or does not change: FallbackDrive LOGO / USB (how the Control Hub is mounted)."));
        steps.add(new Step("Intake A", "Hold RT: the intake LEFT of the shooter pulls a ball IN. Only motor intakeA moves.",
                "The other intake ran: intakeA / intakeB are swapped (John.xml ports, or the cables). Right intake, pushes out: flip JohnIntake.A_REVERSED."));
        steps.add(new Step("Intake B", "Hold RT: the intake RIGHT of the shooter pulls a ball IN. Only motor intakeB moves.",
                "The other intake ran: intakeA / intakeB swapped. Right intake, pushes out: flip JohnIntake.B_REVERSED."));
        steps.add(new Step("Indexer", "Press RT: the gate turns one step (90 deg) the way that feeds a ball to the flywheel. Angle goes up by 90.",
                "Turned backward: flip JohnIndexer.REVERSED. Moved more or less than a quarter turn: TICKS_PER_REV / gearing."));
        steps.add(new Step("Flywheel", "Hold RT (0.3 power): the wheel spins the way that launches a ball OUT, and RPM reads POSITIVE.",
                "Spins backward: flip JohnFlywheel.REVERSED. Right way but RPM negative or 0: encoder cable / sign. "
                        + "New belt: rerun Flywheel Calibration either way."));
        steps.add(new Step("Defender", "RT: the brace toggles between stowed (folded in) and deployed (folded out).",
                "Wrong ends: recalibrate in Defender Calibration. No motion: servo port / config."));
        steps.add(new Step("LEDs", "RT: cycles LED 1 then LED 2 through red, green, amber, off. Colors match the screen.",
                "Red and green swapped: flip JohnStatusLights.LOWER_PORT_IS_RED. Wrong LED lit: digital ports."));
        steps.add(new Step("Camera", "Point the camera wall at the HIVE: HIVE tags listed below.",
                "No frames: Limelight cable / pipeline. Frames but no tags: exposure, or the camera mount moved."));

        while (opModeInInit()) {
            telemetry.addLine("Guided hardware check: " + steps.size() + " steps.");
            telemetry.addLine("RT runs the device; A right, B wrong, X skip.");
            telemetry.addLine("Wheels first: John on a stand.");
            telemetry.update();
        }
        gamepad1.resetEdgeDetection();
        int at = 0;
        int[] driveStart = new int[4];
        int startA = 0, startB = 0;
        boolean triggerWas = false;
        int ledPhase = 0;
        boolean deployed = false;
        com.qualcomm.robotcore.util.ElapsedTime clock = new com.qualcomm.robotcore.util.ElapsedTime();
        int lastAt = -1;

        while (opModeIsActive()) {
            Step step = steps.get(at);
            boolean rt = gamepad1.right_trigger > 0.5;
            boolean rtPressed = rt && !triggerWas;
            triggerWas = rt;
            if (at != lastAt) {
                // Entering a step: references for "what moved"
                for (int i = 0; i < 4; i++) driveStart[i] = drive[i].getCurrentPosition();
                startA = intakeA.getCurrentPosition();
                startB = intakeB.getCurrentPosition();
                lastAt = at;
            }
            String reading = "";
            // Pushed by hand (Pinpoint, IMU): wheels coast; else they brake
            boolean pushing = step.name.startsWith("Pinpoint") || step.name.equals("Hub IMU");
            for (DcMotorEx m : drive) {
                m.setZeroPowerBehavior(pushing ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
            }
            // Everything off unless this step runs it
            for (DcMotorEx m : drive) m.setPower(0);
            intake.off();
            flywheel.setManualPower(0.0);

            if (step.name.startsWith("Wheel ")) {
                int i = 0;
                while (!step.name.equals("Wheel " + driveNames[i])) i++;
                if (rt) drive[i].setPower(DRIVE_POWER);
                StringBuilder sb = new StringBuilder("encoders moved:");
                for (int k = 0; k < 4; k++) {
                    sb.append(String.format(Locale.US, " %s %+d", driveNames[k], drive[k].getCurrentPosition() - driveStart[k]));
                }
                reading = sb.toString();
            } else if (step.name.startsWith("Pinpoint")) {
                if (rtPressed) {
                    follower.setPose(new Pose(0, 0, 0));
                }
                follower.manual(com.pedropathing.drivetrain.DrivePowers.zero());   // no hold: John is pushed
                follower.update();
                for (DcMotorEx m : drive) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Pose p = follower.pose();
                reading = String.format(Locale.US, "X %+.1f in  Y %+.1f in  heading %+.1f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
            } else if (step.name.equals("Hub IMU")) {
                imu.update();
                reading = String.format(Locale.US, "IMU heading %+.1f deg (%s)", Math.toDegrees(imu.heading()), imu.status());
            } else if (step.name.equals("Intake A") || step.name.equals("Intake B")) {
                boolean a = step.name.endsWith("A");
                if (rt) intake.only(a ? Intake.Side.A : Intake.Side.B, INTAKE_POWER);
                reading = String.format(Locale.US, "encoders moved: intakeA %+d, intakeB %+d (0 = no encoder cable)",
                        intakeA.getCurrentPosition() - startA, intakeB.getCurrentPosition() - startB);
            } else if (step.name.equals("Indexer")) {
                if (rtPressed) indexer.step(1);
                indexer.update();
                reading = String.format(Locale.US, "angle %.0f deg, target %.0f, %.2f A, %s", indexer.degrees(),
                        indexer.targetDegrees(), indexer.amps(), indexer.state());
            } else if (step.name.equals("Flywheel")) {
                if (rt) flywheel.setManualPower(FLYWHEEL_POWER);
                reading = String.format(Locale.US, "%.0f RPM (motor %.0f ticks/s)", flywheel.rpm(), flywheel.ticksPerSecond());
            } else if (step.name.equals("Defender")) {
                if (rtPressed) {
                    deployed = !deployed;
                    if (deployed) defender.deploy(); else defender.stow();
                }
                reading = defender.status();
            } else if (step.name.equals("LEDs")) {
                StatusLights.Color[] cycle = {StatusLights.Color.RED, StatusLights.Color.GREEN, StatusLights.Color.AMBER,
                        StatusLights.Color.OFF};
                if (rtPressed) ledPhase = (ledPhase + 1) % 8;
                lights.set(1, ledPhase < 4 ? cycle[ledPhase] : StatusLights.Color.OFF);
                lights.set(2, ledPhase >= 4 ? cycle[ledPhase - 4] : StatusLights.Color.OFF);
                reading = "LED " + (ledPhase < 4 ? 1 : 2) + " should be " + cycle[ledPhase % 4];
            } else if (step.name.equals("Camera")) {
                HiveCamera.Frame f = camera.poll();
                reading = camera.status() + (f == null ? "" : "; tags " + f.ids());
            }
            flywheel.update();

            // Answers and navigation
            Answer given = gamepad1.aWasPressed() ? Answer.OK : gamepad1.bWasPressed() ? Answer.WRONG
                    : gamepad1.xWasPressed() ? Answer.SKIPPED : Answer.NONE;
            if (given != Answer.NONE) {
                step.answer = given;
                log.row(clock.seconds(), step.name, given, reading);
                log.flush();
                at = Math.min(steps.size(), at + 1);
            }
            if (gamepad1.dpadRightWasPressed()) at = Math.min(steps.size(), at + 1);
            if (gamepad1.dpadLeftWasPressed()) at = Math.max(0, at - 1);

            if (at >= steps.size()) {
                // Summary
                for (DcMotorEx m : drive) m.setPower(0);
                intake.off();
                flywheel.setManualPower(0.0);
                flywheel.update();
                telemetry.addLine(">>> SUMMARY (dpad left: back)");
                int ok = 0;
                for (Step s : steps) if (s.answer == Answer.OK) ok++;
                telemetry.addLine(ok + " of " + steps.size() + " right");
                for (Step s : steps) {
                    if (s.answer == Answer.WRONG) {
                        telemetry.addLine("WRONG " + s.name + ": " + s.ifWrong);
                    } else if (s.answer != Answer.OK) {
                        telemetry.addLine("not checked: " + s.name);
                    }
                }
                telemetry.update();
                continue;   // dpad left (above) goes back to the last step
            }
            telemetry.addLine(String.format(Locale.US, ">>> %d/%d %s%s", at + 1, steps.size(), step.name,
                    step.answer == Answer.NONE ? "" : " [" + step.answer + "]"));
            telemetry.addLine(step.expect);
            if (!reading.isEmpty()) telemetry.addLine(reading);
            telemetry.addLine("RT run | A right | B wrong | X skip | dpad < >");
            telemetry.update();
        }
        for (DcMotorEx m : drive) m.setPower(0);
        intake.off();
        flywheel.stop();
        indexer.stop();
        lights.off();
        camera.stop();
        log.close();
    }
}
