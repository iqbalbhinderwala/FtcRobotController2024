package org.firstinspires.ftc.teamcode.Vex.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Vex.Hardware.DecodeField;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexActuators;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="[Vex] Shooter Calibration", group="VexTest")
public class VexShooterCalibration extends LinearOpMode {

    private VexActuators actuators = new VexActuators(this);
    private VexOdometryDriveTrain driveTrain = new VexOdometryDriveTrain(this);
    private VexVision vision = new VexVision(this);

    @Override
    public void runOpMode() {

        actuators.init(hardwareMap);
        driveTrain.init();
        vision.init();

        driveTrain.resetPose(0,0,0);

        actuators.openGateA();
        boolean gateA_is_open = true;

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Driver orientation is set to face the +Y-Axis (0 deg).");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ElapsedTime lastPress = new ElapsedTime();
        final double BUTTON_DELAY = 0.25;

        double shooterTargetRPM = VexActuators.SHOOTER_RPM_MAX / 2;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // --- VISION POSE CORRECTION ---
            AprilTagDetection newDetection = vision.getMostAccurateTarget();
            if (newDetection != null && newDetection.robotPose != null) {
                Pose3D robotPose = newDetection.robotPose;
                driveTrain.updatePose(robotPose.getPosition().x, robotPose.getPosition().y, robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.addLine("!!! POSE UPDATED FROM VISION !!!");
            }

            // --- READ JOYSTICK INPUTS ---

            // Get raw joystick values for movement and turning.
            double forwardInput = -gamepad1.left_stick_y; // Positive is "forward"
            double strafeInput  =  gamepad1.left_stick_x;  // Positive is "right"
            double turnInput    = -gamepad1.right_stick_x; // Positive is counter-clockwise

            // --- CALL DRIVETRAIN METHOD ---

            // Get the robot's current heading from the IMU.
            double robotHeading = driveTrain.getHeading();

            // Define the driver's fixed orientation. 0 degrees means facing along the +Y axis.
            double humanDirection = 0.0;

            // Call the centralized driving function within the drivetrain class,
            // passing all necessary inputs. The drivetrain now handles all calculations.
            driveTrain.moveHumanCentric(forwardInput, strafeInput, turnInput, robotHeading, humanDirection);

            // --- Adjust Shooter ---
            final double SHOOTER_INCREMENT = VexActuators.SHOOTER_RPM_INCREMENT; // 40 RPM per second
            if (gamepad1.dpad_up && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                shooterTargetRPM = Range.clip(shooterTargetRPM + SHOOTER_INCREMENT, 0, VexActuators.SHOOTER_RPM_MAX);
            }
            if (gamepad1.dpad_down && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                shooterTargetRPM = Range.clip(shooterTargetRPM - SHOOTER_INCREMENT, 0, VexActuators.SHOOTER_RPM_MAX);
            }

            // --- Activate Shooter (X/Y Button) ---
            if (gamepad1.x) {
                actuators.setShooterRPM(shooterTargetRPM);
            } else if (gamepad1.y) {
                double dist_INCH = DecodeField.getDistanceToAllianceCorner(DecodeField.Alliance.RED, driveTrain.getPose2D());
                shooterTargetRPM = actuators.predictShooterRPMFromDistance(dist_INCH);
                actuators.setShooterRPM(shooterTargetRPM);
            } else {
                actuators.setShooterRPM(0);
            }

            if (gamepad1.a && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                if (gateA_is_open) {
                    actuators.closeGateA();
                    gateA_is_open = false;
                } else {
                    actuators.openGateA();
                    gateA_is_open = true;
                }
            }

            if (gamepad1.left_stick_button && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                actuators.enablePowerAdjustment = !actuators.enablePowerAdjustment;
            }

            if (gamepad1.left_bumper && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                actuators.powerAdjustementFactor = actuators.powerAdjustementFactor - 0.05;
            }
            if (gamepad1.right_bumper && lastPress.seconds() > BUTTON_DELAY) {
                lastPress.reset();
                actuators.powerAdjustementFactor = actuators.powerAdjustementFactor + 0.05;
            }

            driveTrain.update();

            Pose2D pose = driveTrain.getPose2D();

            telemetry.addData("--- Actuators ---", "");
            telemetry.addData(">", "X: Manual-RPM (DPAD up/down)");
            telemetry.addData(">", "Y: Auto-RPM (VISION)");
            telemetry.addData(">", "A: Toggle Gate");
            telemetry.addData(">", "Left Stick Button: Toggle Power Adjustment");
            telemetry.addData(">", "Left/Right Bumper: Adjust Power Adjustment Factor");

            telemetry.addData("--- Control ---", "");
            telemetry.addData("Power Adjustment State", actuators.enablePowerAdjustment);
            telemetry.addData("Power Adjustment factor", "%.3f", actuators.powerAdjustementFactor);

            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Voltage", "%.2f", actuators.getVoltage());
            telemetry.addData("Shooter Target RPM", "%.2f", shooterTargetRPM);
            telemetry.addData("Shooter Actual RPM", "%.2f", actuators.getShooterRPM());
            telemetry.addData("Shooter Power", "%.2f", actuators.getShooterPower());
            telemetry.addData("Shooter Ready", actuators.isShooterAtTargetRPM(shooterTargetRPM));
            telemetry.addData("Coordinates", "(%.2f, %.2f) inch", pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
            telemetry.addData("Tile Coordinates", "(%.2f, %.2f) TILES", pose.getX(DistanceUnit.INCH)/TILE, pose.getY(DistanceUnit.INCH)/TILE);
            telemetry.addData("Distance to Corner", "%.2f inch", DecodeField.getDistanceToAllianceCorner(DecodeField.Alliance.RED, pose));
            telemetry.addData("Distance to Corner", "%.2f TILES", DecodeField.getDistanceToAllianceCorner(DecodeField.Alliance.RED, pose)/TILE);
            telemetry.update();
        }

        driveTrain.stopMotors();
    }

    final double TILE = 24; // inches
}
