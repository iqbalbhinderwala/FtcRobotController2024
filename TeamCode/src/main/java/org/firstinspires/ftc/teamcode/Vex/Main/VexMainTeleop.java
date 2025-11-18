package org.firstinspires.ftc.teamcode.Vex.Main;

import android.util.Log;
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
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexBlackboard;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * This is the main TeleOp program for the Vex robot, structured like an Iterative OpMode
 * but implemented within a LinearOpMode for robust, single-threaded execution.
 *
 * It combines human-centric driving with full control over all actuators and utilizes
 * AprilTag vision for pose initialization and in-match correction.
 *
 * CONTROLS:
 * - Left Stick:  Controls strafing and forward/backward movement from the driver's perspective.
 * - Left Stick Button: Press to halve the driving power for finer control.
 * - Right Stick: Controls the robot's rotation (turning).
 * - Right Stick Button: Press to automatically align to the alliance corner for shooting.
 * - Gamepad 1 Left or Right Bumper: Runs the intake.
 * - Gamepad 1 'B': Reverses the intake.
 * - Triggers: Controls the shooting mechanism.
 *   - One trigger pulled: Spins up the shooter wheels, gates in ready state (A closed, B open).
 *   - Both triggers pulled: Activates the gate cycle to shoot (after a 1-second spin-up).
 * - D-Pad Up/Down: Increases/decreases the shooter power by a small increment to adjust for battery level.
 * - D-Pad Left/Right: Resets the shooter power adjustment to zero.
 */
@TeleOp(name = "[Vex] Main TeleOp", group = "Vex")
public class VexMainTeleop extends LinearOpMode {
    private static final String TAG = "VEX::MainTele";

    // Hardware Classes
    private VexOdometryDriveTrain driveTrain;
    private VexVision vision;
    private VexActuators actuators;
    private VexBlackboard blackboardHelper;

    // OpMode Members
    private ElapsedTime lastPress;
    private double shooterRPM = VexActuators.SHOOTER_RPM_LOW;
    private double shooterRPMAdjustment = 0.0; // Adjustment for shooter RPM
    private double humanDirection;

    // Alliance information
    private DecodeField.Alliance currentAlliance;

    // Shooting state machine
    private enum ShootingState { IDLE, SPIN_UP, SHOOTING_CYCLE }
    private ShootingState shootingState = ShootingState.IDLE;
    private enum GateCycleState { READY, STEP_1_CLOSE_B, STEP_2_OPEN_A, STEP_3_CLOSE_A, STEP_4_OPEN_B }
    private GateCycleState gateCycleState = GateCycleState.READY;
    private ElapsedTime gateCycleTimer; // For timing gate movements
    private ElapsedTime spinUpTimer;    // For timing wheel spin-up

    private double targetHeading;
    private ElapsedTime visionUpdateTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Run initialization code once
        opModeInit();

        // Run the initialization loop until the user presses Start
        while (!isStarted() && !isStopRequested()) {
            opModeInitLoop();
        }

        if (isStopRequested()) return;

        // Run the start routine once
        opModeStart();

        // Run the main TeleOp loop
        while (opModeIsActive()) {
            opModeLoop();
        }

        // Run the cleanup routine once
        opModeStop();
    }

    /**
     * Replicates the init() method of an Iterative OpMode. This code is run once
     * when the OpMode is selected on the Driver Station.
     */
    private void opModeInit() {
        Log.d(TAG, "opModeInit: Initializing...");
        // Initialize hardware classes
        blackboardHelper = new VexBlackboard(this);
        driveTrain = new VexOdometryDriveTrain(this);
        actuators = new VexActuators(this);
        vision = new VexVision(this);

        // Add blackboard telemetry
        blackboardHelper.addBlackboardTelemetry();

        driveTrain.init();
        actuators.init(hardwareMap);
        vision.init();

        actuators.closeGateA(); // Start with Gate A closed
        actuators.openGateB();  // and Gate B open (ready for intake)

        // Read alliance selection from the blackboard
        currentAlliance = blackboardHelper.getAlliance();

        // Initialize state variables
        lastPress = new ElapsedTime();
        gateCycleTimer = new ElapsedTime();
        spinUpTimer = new ElapsedTime();
        visionUpdateTimer = new ElapsedTime();

        // Set humanDirection based on the alliance: 0 for RED, 180 for BLUE
        humanDirection = (currentAlliance == DecodeField.Alliance.RED) ? 0 : 180;

        Log.d(TAG, "opModeInit: Initialization complete.");
    }

    /**
     * Replicates the init_loop() method of an Iterative OpMode. This code is looped
     * after init() and before start().
     */
    private void opModeInitLoop() {
        if (currentAlliance == DecodeField.Alliance.UNKNOWN) {
            telemetry.addData("ERROR", "Alliance not selected! Please select an alliance in [Vex] GameSetup TeleOp.");
        } else {
            telemetry.addData("Alliance", currentAlliance.toString());
        }
        // Add blackboard telemetry
        blackboardHelper.addBlackboardTelemetry();

        telemetry.addData("--- Status ---", "Initialization Complete.");
        telemetry.addData(">", "Searching for AprilTag... Press PLAY to start without one.");

        AprilTagDetection detection = vision.getMostAccurateTarget();
        if (detection != null && detection.robotPose != null) {
            telemetry.addLine("AprilTag found! Pose will be updated from this tag.");
            vision.addTelemetry();
        } else {
            telemetry.addLine("Searching for AprilTag...");
        }

        telemetry.update();
        sleep(50); // Small sleep to prevent busy-looping
    }

    /**
     * Replicates the start() method of an Iterative OpMode. This code is run once
     * when the PLAY button is pressed.
     */
    private void opModeStart() {
        Log.d(TAG, "opModeStart: Starting...");
        lastPress.reset(); // Reset timer on start
        gateCycleTimer.reset();
        spinUpTimer.reset();
        visionUpdateTimer.reset();

        // Try to initialize pose from the blackboard first
        Pose2D storedPose = blackboardHelper.getPose();
        if (storedPose != null) {
            driveTrain.resetPose(storedPose.getX(DistanceUnit.INCH), storedPose.getY(DistanceUnit.INCH), storedPose.getHeading(AngleUnit.DEGREES));
            Log.d(TAG, "opModeStart: Pose initialized from blackboard: " + storedPose);
            telemetry.addLine("Pose initialized from final pose of last OpMode.");
        } else {
            // If not, try to use the starting position from the setup
            DecodeField.KeyLocation startingLocation = blackboardHelper.getStartingLocation();
            if (startingLocation != DecodeField.KeyLocation.UNKNOWN) {
                Pose2D startingPose = startingLocation.getPose();
                driveTrain.resetPose(startingPose.getX(DistanceUnit.INCH), startingPose.getY(DistanceUnit.INCH), startingPose.getHeading(AngleUnit.DEGREES));
                Log.d(TAG, "opModeStart: Pose initialized from starting location: " + startingLocation);
                telemetry.addData("Pose initialized from", startingLocation.toString());
            } else {
                // If no blackboard pose, try to set initial pose from AprilTag
                AprilTagDetection detection = vision.getMostAccurateTarget();
                if (detection != null && detection.robotPose != null) {
                    Pose3D robotPose = detection.robotPose;
                    driveTrain.resetPose(robotPose.getPosition().x, robotPose.getPosition().y, robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                    Log.d(TAG, "opModeStart: Pose initialized from AprilTag: " + robotPose);
                    telemetry.addLine("Pose initialized from AprilTag.");
                } else {
                    driveTrain.resetPose(0, 0, 90);
                    Log.d(TAG, "opModeStart: No pose source found, starting at (0,0,90).");
                    telemetry.addLine("No AprilTag, starting position, or stored pose found, starting at (0,0,NORTH).");
                }
            }
        }

        telemetry.addData(">", "TeleOp Started. Driver orientation is set to face the +Y-Axis (0 deg).");
        telemetry.update();
        Log.d(TAG, "opModeStart: Start complete.");
    }


    /**
     * Replicates the loop() method of an Iterative OpMode. This code is looped
     * continuously after start() until stop() is called.
     */
    private void opModeLoop() {
        // --- VISION POSE CORRECTION ---
        AprilTagDetection newDetection = vision.getMostAccurateTarget();
        if (newDetection != null && newDetection.robotPose != null) {
            Pose3D robotPose = newDetection.robotPose;

            // - UPDATE HEADING ONCE PER SECOND: Only update the robot's orientation if 1 second has passed.
            if (visionUpdateTimer.seconds() > VISION_POSE_UPDATE_INTERVAL_SECONDS) {
                visionUpdateTimer.reset(); // Reset the timer after the update
                driveTrain.updatePose(robotPose.getPosition().x, robotPose.getPosition().y, robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                Log.d(TAG, "opModeLoop: Pose updated from vision: " + robotPose);
                telemetry.addLine("!!! HEADING UPDATED FROM VISION !!!");
            } else {
                // - UPDATE POSITION ALWAYS: Continuously update the robot's X and Y coordinates.
                driveTrain.updatePosition(robotPose.getPosition().x, robotPose.getPosition().y);
                Log.d(TAG, "opModeLoop: Position updated from vision: " + robotPose);
                telemetry.addLine("!!! POSITION UPDATED FROM VISION !!!");
            }
        }

        // --- DRIVING CONTROLS ---
        driveAndMove();

        // --- ACTUATOR CONTROLS ---
        handleShooting();
        handleActuators();

        // --- TELEMETRY ---
        updateTelemetry();
    }

    /**
     * Replicates the stop() method of an Iterative OpMode. This code is run once
     * when the OpMode is stopped.
     */
    private void opModeStop() {
        Log.d(TAG, "opModeStop: Stopping...");
        blackboardHelper.resetPose();

        driveTrain.stopMotors();
        vision.stop();
        actuators.setShooterPower(0);
        actuators.setIntakePower(0);
        telemetry.addData("Status", "OpMode Stopped.");
        telemetry.update();
        Log.d(TAG, "opModeStop: Stop complete.");
    }

    /**
     * Handles all driving and movement logic.
     */
    private void driveAndMove() {
        double forwardInput = -gamepad1.left_stick_y * MAX_DRIVE_SPEED;
        double strafeInput  =  gamepad1.left_stick_x * MAX_STRAFE_SPEED;
        double turnInput = -gamepad1.right_stick_x * MAX_TURN_SPEED;

        // When the right stick is pressed, override manual turning to auto-align.
        if (gamepad1.right_stick_button) {
            double turnPower = calculateAutoAlignTurnPower();
            turnInput = Range.clip(turnPower, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            Log.d(TAG, "driveAndMove: Auto-aligning with turn power: " + turnInput);
        }

        // If left stick is pressed, halve all driving power for finer control.
        if (gamepad1.left_stick_button) {
            forwardInput *= 0.5;
            strafeInput  *= 0.5;
            turnInput    *= 0.5;
        }

        double robotHeading = driveTrain.getHeading();

        driveTrain.moveHumanCentric(forwardInput, strafeInput, turnInput, robotHeading, humanDirection);
    }

    /**
     * Calculates the auto-alignment turn power.
     * - If the shooter is spinning, it aligns to the alliance corner.
     * - If the shooter is idle, it aligns to a fixed heading of 90 degrees.
     * @return A turn power value, from -1.0 to 1.0.
     */
    private double calculateAutoAlignTurnPower() {
        double currentHeading = driveTrain.getHeading();

        // Align to the alliance corner
        Log.d(TAG, "calculateAutoAlignTurn: Auto-Aligning to " + currentAlliance.toString() + " Corner");

        double headingError = DecodeField.getTurnAngleToAllianceCorner(currentAlliance, driveTrain.getPose2D());
        if (Math.abs(headingError) < TURN_TOLERANCE_DEGREES) {
            return 0.0;
        }

        targetHeading = driveTrain.getHeading() + headingError;

        // Use the helper function to calculate the turn power.
        return driveTrain.calculateTurnPower(targetHeading, currentHeading);
    }

    /**
     * Handles all actuator logic (intake, shooter, gates).
     */
    private void handleActuators() {
        // --- Shooter Power Adjustment ---
        // Use the D-Pad to fine-tune the shooter power. Reset with D-Pad Left/Right.
        if (gamepad1.dpad_up && lastPress.seconds() > BUTTON_DELAY) {
            lastPress.reset();
            shooterRPMAdjustment += RPM_INCREMENT;
        } else if (gamepad1.dpad_down && lastPress.seconds() > BUTTON_DELAY) {
            lastPress.reset();
            shooterRPMAdjustment -= RPM_INCREMENT;
        } else if ((gamepad1.dpad_left || gamepad1.dpad_right) && lastPress.seconds() > BUTTON_DELAY) {
            lastPress.reset();
            shooterRPMAdjustment = 0.0; // Reset the offset
        }

        // Manual controls are disabled during auto-shoot sequence
        if (shootingState == ShootingState.IDLE) {
            actuators.setShooterPower(0);
        }

        // Intake Control
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            actuators.setIntakePower(MAX_INTAKE_POWER);
            if (shootingState != ShootingState.SHOOTING_CYCLE) {
                actuators.closeGateA();
                actuators.openGateB();
            }
        } else if (gamepad1.b) {
            actuators.setIntakePower(-MAX_INTAKE_POWER);
        } else {
            actuators.setIntakePower(0);
        }
    }

    private void handleShooting() {
        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        boolean anyTrigger = leftTrigger || rightTrigger;
        boolean bothTriggers = leftTrigger && rightTrigger;

        ShootingState lastState = shootingState;

        // Keep shooter RPM target value always updated for telemetry
        updateShooterTargetRPMFromDistance();

        switch (shootingState) {
            case IDLE:
                if (anyTrigger) {
                    // Transition to SPIN_UP
                    shootingState = ShootingState.SPIN_UP;
                    spinUpTimer.reset(); // Start the spin-up timer

                    actuators.setShooterRPM(shooterRPM);

                    actuators.closeGateA(); // Set gates to ready-to-shoot state
                    actuators.openGateB();
                }
                break;
            case SPIN_UP:
                if (!anyTrigger) {
                    // Transition back to IDLE
                    shootingState = ShootingState.IDLE;
                    actuators.setShooterPower(0);
                    actuators.closeGateA(); // Set gates to ready-for-intake
                    actuators.openGateB();
                } else if (bothTriggers && spinUpTimer.seconds() >= SPIN_UP_TIME_S) {
                    if (!DecodeField.isInRangeForShooting(currentAlliance, driveTrain.getPose2D())) {
                        gamepad1.rumble(500);
                    } else {
                        // Spin-up complete, transition to SHOOTING_CYCLE
                        shootingState = ShootingState.SHOOTING_CYCLE;
                        gateCycleState = GateCycleState.STEP_1_CLOSE_B; // Start the cycle
                        gateCycleTimer.reset(); // Reset timer for the first gate delay
                    }
                }
                break;
            case SHOOTING_CYCLE:
                if (!bothTriggers) {
                    // Released one or both triggers, go back to spinning up.
                    // The spinUpTimer is NOT reset, as the wheels are still spinning.
                    shootingState = ShootingState.SPIN_UP;
                    gateCycleState = GateCycleState.READY;
                    actuators.closeGateA(); // Reset gates to ready-to-shoot state
                    actuators.openGateB();
                } else {
                    runShootingCycleIteration();
                }
                break;
        }

        if (lastState != shootingState) {
            Log.d(TAG, "handleShooting: State changed from " + lastState + " to " + shootingState);
        }
    }

    private void updateShooterTargetRPMFromDistance() {
        // Automatically determine shooter RPM based on distance
        double distanceToCorner = DecodeField.getDistanceToAllianceCorner(currentAlliance, driveTrain.getPose2D());
        double predictedShooterRPM = actuators.predictShooterRPMFromDistance(distanceToCorner);

        // Apply the offset and clip the value to a safe range [0, 2000]
        shooterRPM = Range.clip(predictedShooterRPM + shooterRPMAdjustment, 0, VexActuators.SHOOTER_RPM_MAX);
    }

    private void runShootingCycleIteration() {
        // Use the dedicated gate timer
        if (gateCycleState != GateCycleState.READY && gateCycleTimer.milliseconds() < GATE_DELAY_MS) {
            return; // Wait for delay
        }

        gateCycleTimer.reset(); // Reset timer for the next step in the cycle
        GateCycleState lastState = gateCycleState;

        switch (gateCycleState) {
            case STEP_1_CLOSE_B:
                actuators.closeGateB();
                    gateCycleState = GateCycleState.STEP_2_OPEN_A;
                break;
            case STEP_2_OPEN_A:
                actuators.openGateA();
                    gateCycleState = GateCycleState.STEP_3_CLOSE_A;
                break;
            case STEP_3_CLOSE_A:
                actuators.closeGateA();
                    gateCycleState = GateCycleState.STEP_4_OPEN_B;
                break;
            case STEP_4_OPEN_B:
                actuators.openGateB();
                gateCycleState = GateCycleState.STEP_1_CLOSE_B; // Loop back
                break;
            default:
                // Should not happen, but as a safe-guard, reset to ready
                gateCycleState = GateCycleState.READY;
                break;
        }

        if (lastState != gateCycleState) {
            Log.d(TAG, "runShootingCycleIteration: State changed from " + lastState + " to " + gateCycleState);
        }
    }

    /**
     * Updates and displays all telemetry data on the Driver Station.
     */
    private void updateTelemetry() {
        driveTrain.update(); // Update odometry
        Pose2D currentPose = driveTrain.getPose2D();

        telemetry.addData("--- Robot ---", "");
        telemetry.addData("Pose", "X: %.2f, Y: %.2f (in), H: %.1f (deg)",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Dist to Corner", "%.2f in", DecodeField.getDistanceToAllianceCorner(currentAlliance, currentPose));
        telemetry.addData("Target Heading", "%.2f in", targetHeading);

        telemetry.addData("--- Actuators ---", "");
//        telemetry.addData("Shooting State", shootingState.toString());
//        telemetry.addData("Gate Cycle", gateCycleState.toString());
//        telemetry.addData("Intake Power", "%.2f", MAX_INTAKE_POWER);
        telemetry.addData("Shooter Target RPM", "%.1f", shooterRPM);
        telemetry.addData("** Shooter RPM Adjustment **", "%.1f", shooterRPMAdjustment); // Display the adjustment
//        telemetry.addData("Gate A Pos", "%.2f", actuators.getGateAPosition());
//        telemetry.addData("Gate B Pos", "%.2f", actuators.getGateBPosition());

        vision.addTelemetry();

        telemetry.update();
    }

    private final double BUTTON_DELAY = 0.25;

    final double MAX_DRIVE_SPEED  = 1.0;
    final double MAX_STRAFE_SPEED = 1.0;
    final double MAX_TURN_SPEED   = 0.6;
    final double TURN_TOLERANCE_DEGREES = 5.0;

    final double MAX_INTAKE_POWER = 1.0;
    private final double RPM_INCREMENT = VexActuators.SHOOTER_RPM_INCREMENT; // 40
    private final double SPIN_UP_TIME_S = 1.25;
    private final long GATE_DELAY_MS = 350;

    private static final double VISION_POSE_UPDATE_INTERVAL_SECONDS = 1.0; // Seconds
}
