package org.firstinspires.ftc.teamcode.John.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.John.Dashboard.Drawing;
import org.firstinspires.ftc.teamcode.John.Pedro.Constants;
import org.firstinspires.ftc.teamcode.John.Pedro.OdometryWatchdog;
import org.firstinspires.ftc.teamcode.John.Util.CsvLog;

/**
 * Manual drive test for John, field-centric (human-centric) by default, with Pedro's
 * ManualDrive.fieldCentric and ManualDrive.driveOrHold.
 *
 * Field-centric: pushing the left stick away from you drives the robot away from you, whichever way
 * it faces. The drivers stand at their alliance wall: poses are FTC field coordinates (+X toward
 * the audience, +Y toward blue), so a red driver (red wall, -Y side) faces +Y and a blue driver
 * (blue wall, +Y side) faces -Y.
 *
 * INIT: B = red, X = blue. Place the robot facing straight away from you, then START: the robot
 * starts at the alliance's start pose facing away from the driver. (It does not take over the pose
 * from [John] Auto yet.)
 *
 * gamepad1:
 *   left stick      drive (field- or robot-centric)
 *   right stick X   turn
 *   right bumper    slow (hold down)
 *   left bumper     toggle field-centric / robot-centric
 *   Y               re-sync the heading: "the robot now faces straight away from me" (after a
 *                   heading jump, or if it was not placed that way at START)
 *   A               toggle hold-when-released: when the sticks are released and the robot has
 *                   stopped, it holds that pose (driveOrHold) instead of just braking
 *
 * OdometryWatchdog stops the drive on bad Pinpoint reads (resumes after a short glitch).
 * Logged to /sdcard/FIRST/john_logs/r<run>_john_drive_test_<time>.csv.
 */
@Config
@TeleOp(name = "[John] Drive Test", group = "JohnTest")
public class DriveTest extends LinearOpMode {

    public enum Alliance { RED, BLUE }

    public static Alliance ALLIANCE = Alliance.RED;
    /** Extra turn of the driver's direction, degrees counter-clockwise (0 = facing straight across). */
    public static double DRIVER_FACING_OFFSET_DEG = 0.0;
    public static double DRIVE_SCALE = 1.0;
    public static double TURN_SCALE = 0.8;
    public static double SLOW_SCALE = 0.35;
    public static double STICK_DEADBAND = 0.05;
    public static boolean HOLD_WHEN_RELEASED = true;
    /** Start position, FTC field coordinates; mirrored in Y for blue. Heading = away from the driver. */
    public static double START_X_IN = 0.0;
    public static double START_Y_IN_RED = -36.0;

    private static final String LOG_HEADER = "t_s,alliance,field_centric,hold_mode,mode,stick_fwd,stick_left,"
            + "stick_turn,slow,x,y,h_deg,watchdog,pinpoint_status,p_fl,p_fr,p_bl,p_br,event";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Follower follower = Constants.create(hardwareMap);
        OdometryWatchdog watchdog = new OdometryWatchdog(hardwareMap, "drive_test");
        DcMotor[] motors = {
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get()),
                hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get())};
        CsvLog log = new CsvLog("john_drive_test", LOG_HEADER);

        while (opModeInInit()) {
            if (gamepad1.b) {
                ALLIANCE = Alliance.RED;
            } else if (gamepad1.x) {
                ALLIANCE = Alliance.BLUE;
            }
            telemetry.addData("Alliance", "%s   (B = red, X = blue): drive from the %s alliance wall",
                    ALLIANCE, ALLIANCE == Alliance.RED ? "red" : "blue");
            telemetry.addLine("Place the robot facing straight away from you, then START.");
            telemetry.addData("Log", log.error().isEmpty() ? log.fileName() : "ERROR " + log.error());
            telemetry.update();
        }
        if (isStopRequested()) {
            log.close();
            return;
        }

        double startY = ALLIANCE == Alliance.RED ? START_Y_IN_RED : -START_Y_IN_RED;
        follower.setPose(new Pose(START_X_IN, startY, driverFacing()));
        watchdog.poseSet();
        follower.update();

        boolean fieldCentric = true;
        boolean holdMode = HOLD_WHEN_RELEASED;
        gamepad1.resetEdgeDetection();   // ignore presses made during INIT
        log.row(0.0, ALLIANCE, 1, holdMode ? 1 : 0, follower.mode(), null, null, null, null,
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()),
                watchdog.state(), watchdog.status(), null, null, null, null, "start");
        ElapsedTime clock = new ElapsedTime();

        while (opModeIsActive()) {
            String event = null;
            boolean leftBumper = gamepad1.leftBumperWasPressed(), y = gamepad1.yWasPressed(), a = gamepad1.aWasPressed();
            if (leftBumper) {
                fieldCentric = !fieldCentric;
                event = fieldCentric ? "field-centric" : "robot-centric";
            }
            if (y) {
                Pose p = follower.pose();
                follower.setPose(new Pose(p.x(), p.y(), driverFacing()));
                watchdog.poseSet();
                follower.manual(DrivePowers.zero());   // drop any hold of the old heading
                event = "heading re-synced";
            }
            if (a) {
                holdMode = !holdMode;
                if (!holdMode) {
                    follower.manual(DrivePowers.zero());
                }
                event = holdMode ? "hold when released ON" : "hold when released OFF";
            }

            boolean slow = gamepad1.right_bumper;
            double scale = slow ? SLOW_SCALE : 1.0;
            double stickFwd = deadband(-gamepad1.left_stick_y);
            double stickLeft = deadband(-gamepad1.left_stick_x);
            double stickTurn = deadband(-gamepad1.right_stick_x);
            double fwd = stickFwd * DRIVE_SCALE * scale;
            double left = stickLeft * DRIVE_SCALE * scale;
            double turn = stickTurn * TURN_SCALE * scale;

            Pose pose = follower.pose();
            DrivePowers powers = fieldCentric
                    // Pedro rotates the stick vector by -(heading + offset): offset = -(driver's facing)
                    ? ManualDrive.fieldCentric(fwd, left, turn, pose.heading(), -driverFacing())
                    : new DrivePowers(fwd, left, turn);
            if (holdMode) {
                ManualDrive.driveOrHold(follower, powers);
            } else {
                follower.manual(powers);
            }
            follower.update();
            OdometryWatchdog.State state = watchdog.check(follower);
            pose = follower.pose();

            if (watchdog.lost()) {
                telemetry.addLine("ODOMETRY LOST, drive stopped: " + watchdog.reason());
            } else if (!watchdog.ok()) {
                telemetry.addLine("ODOMETRY PAUSED: " + watchdog.reason());
            }
            telemetry.addData("Drive", "%s%s, %s", fieldCentric ? "FIELD-centric" : "ROBOT-centric",
                    slow ? " (slow)" : "", holdMode ? "holds when released" : "brakes when released");
            telemetry.addData("Alliance", "%s (driver faces %.0f deg)", ALLIANCE, Math.toDegrees(driverFacing()));
            telemetry.addData("Pose (FTC)", Drawing.format(pose));
            telemetry.addData("Follower", "%s", follower.mode());
            telemetry.addData("Pinpoint", "%s, %s", state, watchdog.status());
            telemetry.addLine("LB field/robot-centric, RB slow, Y re-sync heading, A hold on/off");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket(Drawing.drawDefaultField());
            Drawing.drawField(packet.fieldOverlay());
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            dashboard.sendTelemetryPacket(packet);

            log.row(clock.seconds(), ALLIANCE, fieldCentric ? 1 : 0, holdMode ? 1 : 0, follower.mode(),
                    stickFwd, stickLeft, stickTurn, slow ? 1 : 0, pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                    state, watchdog.status(), motors[0].getPower(), motors[1].getPower(), motors[2].getPower(),
                    motors[3].getPower(), event);
        }
        log.close();
        watchdog.close();
    }

    /** Field direction the driver looks along (FTC, radians): red +90 deg, blue -90 deg, plus the offset. */
    private static double driverFacing() {
        double base = ALLIANCE == Alliance.RED ? 90.0 : -90.0;
        return Math.toRadians(base + DRIVER_FACING_OFFSET_DEG);
    }

    private static double deadband(double v) {
        return Math.abs(v) < STICK_DEADBAND ? 0.0 : v;
    }
}
