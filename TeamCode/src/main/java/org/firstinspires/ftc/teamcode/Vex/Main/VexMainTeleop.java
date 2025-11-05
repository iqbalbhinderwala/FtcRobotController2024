package org.firstinspires.ftc.teamcode.Vex.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
 * - Right Stick Button: Press to automatically align. Aligns to corner if shooting, else aligns to the closest 90 degrees.
 * - Gamepad 1 Right Bumper: Runs the intake.
 * - Gamepad 1 'B': Reverses the intake.
 * - Triggers: Controls the shooting mechanism.
 *   - One trigger pulled: Spins up the shooter wheels, gates in ready state (A closed, B open).
 *   - Both triggers pulled: Activates the gate cycle to shoot (after a 1-second spin-up).
 */
@TeleOp(name = "Vex Main TeleOp", group = "Vex")
public class VexMainTeleop extends LinearOpMode {

    // Hardware Classes
    private VexOdometryDriveTrain driveTrain;
    private VexVision vision;
    private VexActuators actuators;
    private VexBlackboard blackboardHelper;

    // OpMode Members
    private ElapsedTime lastPress;
    private double intakePower = 1.0;
    private double shooterPower = 0.6;
    private double humanDirection;
    private final double BUTTON_DELAY = 0.25;
    private final double POWER_INCREMENT = 0.05;

    // Alliance information
    private VexBlackboard.Alliance currentAlliance;

    // Alliance corner coordinates (in inches).
    // It's assumed that the odometry system uses inches as its unit.
    private static final double RED_CORNER_X = -3.0 * 24.0; // -72 inches
    private static final double RED_CORNER_Y = 3.0 * 24.0;  // +72 inches
    private static final double BLUE_CORNER_X = -3.0 * 24.0; // -72 inches
    private static final double BLUE_CORNER_Y = -3.0 * 24.0; // -72 inches

    // Shooting state machine
    private enum ShootingState { IDLE, SPIN_UP, SHOOTING_CYCLE }
    private ShootingState shootingState = ShootingState.IDLE;
    private enum GateCycleState { READY, STEP_1_CLOSE_B, STEP_2_OPEN_A, STEP_3_CLOSE_A, STEP_4_OPEN_B }
    private GateCycleState gateCycleState = GateCycleState.READY;
    private ElapsedTime gateCycleTimer; // For timing gate movements
    private ElapsedTime spinUpTimer;    // For timing wheel spin-up
    private final long GATE_DELAY_MS = 250;
    private final double SPIN_UP_TIME_S = 1.0;


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
        // Initialize hardware classes
        driveTrain = new VexOdometryDriveTrain(this);
        vision = new VexVision(this);
        actuators = new VexActuators(this);
        blackboardHelper = new VexBlackboard(this);

        driveTrain.init();
        vision.init();
        actuators.init(hardwareMap);
        actuators.closeGateA(); // Start with Gate A closed
        actuators.openGateB();  // and Gate B open (ready for intake)

        // Read alliance selection from the blackboard
        currentAlliance = blackboardHelper.getAlliance();

        // Initialize state variables
        lastPress = new ElapsedTime();
        gateCycleTimer = new ElapsedTime();
        spinUpTimer = new ElapsedTime();
        humanDirection = 0; // Driver faces +Y axis

        if (currentAlliance == VexBlackboard.Alliance.UNKNOWN) {
            telemetry.addData("ERROR", "Alliance not selected! Please select an alliance in [Vex] Game Setup.");
        } else {
            telemetry.addData("Alliance", currentAlliance.toString());
        }
        telemetry.addData("Status", "Initialization Complete.");
        telemetry.addData(">", "Searching for AprilTag... Press PLAY to start without one.");
        telemetry.update();
    }

    /**
     * Replicates the init_loop() method of an Iterative OpMode. This code is looped
     * after init() and before start().
     */
    private void opModeInitLoop() {
        AprilTagDetection detection = vision.getMostAccurateTarget();
        if (detection != null && detection.robotPose != null) {
            telemetry.addLine("AprilTag found! Pose will be initialized from this tag.");
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
        lastPress.reset(); // Reset timer on start
        gateCycleTimer.reset();
        spinUpTimer.reset();

        // Attempt to set initial pose from AprilTag one last time
        AprilTagDetection detection = vision.getMostAccurateTarget();
        if (detection != null && detection.robotPose != null) {
            Pose3D robotPose = detection.robotPose;
            driveTrain.resetPose(robotPose.getPosition().x, robotPose.getPosition().y, robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.addLine("Pose initialized from AprilTag.");
        } else {
            driveTrain.resetPose(0, 0, 0);
            telemetry.addLine("No AprilTag found, starting at (0,0,0).");
        }
        telemetry.addData(">", "TeleOp Started. Driver orientation is set to face the +Y-Axis (0 deg).");
        telemetry.update();
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
            driveTrain.updatePosition(robotPose.getPosition().x, robotPose.getPosition().y);
            telemetry.addLine("!!! POSE UPDATED FROM VISION !!!");
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
        driveTrain.stopMotors();
        vision.stop();
        actuators.setShooterPower(0);
        actuators.setIntakePower(0);
        telemetry.addData("Status", "OpMode Stopped.");
        telemetry.update();
    }

    /**
     * Handles all driving and movement logic.
     */
    private void driveAndMove() {
        double forwardInput = -gamepad1.left_stick_y;
        double strafeInput  =  gamepad1.left_stick_x;
        double turnInput;

        // When the right stick is pressed, override manual turning to auto-align.
        if (gamepad1.right_stick_button) {
            turnInput = calculateAutoAlignTurn();
        } else {
            turnInput = -gamepad1.right_stick_x;
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
    private double calculateAutoAlignTurn() {
        double targetHeading;
        double currentHeading = driveTrain.getHeading();

        // Check if the shooter wheels are spinning
        if (shootingState != ShootingState.IDLE) {
            // Align to the alliance corner
            telemetry.addData("Auto-Aligning to", currentAlliance.toString() + " Corner");

            // 1. Get the target coordinates based on the current alliance.
            double targetX = (currentAlliance == VexBlackboard.Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
            double targetY = (currentAlliance == VexBlackboard.Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

            // 2. Get the robot's current pose.
            Pose3D currentPose = driveTrain.getPose();
            double currentX = currentPose.getPosition().x;
            double currentY = currentPose.getPosition().y;

            // 3. Calculate the angle to the target.
            targetHeading = Math.toDegrees(Math.atan2(targetY - currentY, targetX - currentX));
        } else {
            // Align to the closest 90-degree heading
            targetHeading = Math.round(currentHeading / 90.0) * 90.0;
            telemetry.addData("Auto-Aligning to", String.format("%.0f Degrees", targetHeading));
        }

        // Use the helper function to calculate the turn power.
        return driveTrain.calculateTurnPower(targetHeading, currentHeading);
    }
    
    /**
     * Calculates the distance from the robot to the current alliance corner.
     * @return The distance in inches.
     */
    private double getDistanceToCorner() {
        // 1. Get the target coordinates based on the current alliance.
        double targetX = (currentAlliance == VexBlackboard.Alliance.RED) ? RED_CORNER_X : BLUE_CORNER_X;
        double targetY = (currentAlliance == VexBlackboard.Alliance.RED) ? RED_CORNER_Y : BLUE_CORNER_Y;

        // 2. Get the robot's current pose.
        Pose3D currentPose = driveTrain.getPose();
        double currentX = currentPose.getPosition().x;
        double currentY = currentPose.getPosition().y;

        // 3. Calculate the distance using the distance formula.
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        return Math.sqrt(dx*dx + dy*dy);
    }

    /**
     * Calculates the shooter power based on the distance to the alliance corner.
     * The formula is power(x in) = (4.48 / 24 * x + 55.055) / 100.
     * @return The calculated shooter power, a value between 0.0 and 1.0.
     */
    private double calculateDistanceBasedShooterPower() {
        double distanceInInches = getDistanceToCorner();
        double power = (4.48 / 24.0 * distanceInInches + 55.055) / 100.0;
        // Clamp the power to be between 0.0 and 1.0, which is what the motor can take.
        return Math.max(0.0, Math.min(1.0, power));
    }

    /**
     * Handles all actuator logic (intake, shooter, gates).
     */
    private void handleActuators() {
        // Manual controls are disabled during auto-shoot sequence
        if (shootingState == ShootingState.IDLE) {
            actuators.setShooterPower(0);

            // Intake Control
            if (gamepad1.right_bumper) {
                actuators.setIntakePower(intakePower);
                actuators.closeGateA();
                actuators.openGateB();
            } else if (gamepad1.b) {
                actuators.setIntakePower(-intakePower);
            } else {
                actuators.setIntakePower(0);
            }
        }
    }

    private void handleShooting() {
        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        boolean anyTrigger = leftTrigger || rightTrigger;
        boolean bothTriggers = leftTrigger && rightTrigger;

        switch (shootingState) {
            case IDLE:
                if (anyTrigger) {
                    // Transition to SPIN_UP
                    shootingState = ShootingState.SPIN_UP;
                    spinUpTimer.reset(); // Start the spin-up timer

                    // Automatically determine shooter power based on distance
                    shooterPower = calculateDistanceBasedShooterPower();
                    actuators.setShooterPower(shooterPower);

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
                    // Spin-up complete, transition to SHOOTING_CYCLE
                    shootingState = ShootingState.SHOOTING_CYCLE;
                    gateCycleState = GateCycleState.STEP_1_CLOSE_B; // Start the cycle
                    gateCycleTimer.reset(); // Reset timer for the first gate delay
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
    }

    private void runShootingCycleIteration() {
        // Use the dedicated gate timer
        if (gateCycleState != GateCycleState.READY && gateCycleTimer.milliseconds() < GATE_DELAY_MS) {
            return; // Wait for delay
        }

        gateCycleTimer.reset(); // Reset timer for the next step in the cycle

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
    }


    /**
     * Updates and displays all telemetry data on the Driver Station.
     */
    private void updateTelemetry() {
        driveTrain.update(); // Update odometry
        Pose3D currentPose = driveTrain.getPose();

        telemetry.addData("--- Robot ---", "");
        telemetry.addData("Pose", "X: %.2f, Y: %.2f (in), H: %.1f (deg)",
                currentPose.getPosition().x,
                currentPose.getPosition().y,
                currentPose.getOrientation().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Dist to Corner", "%.2f in", getDistanceToCorner());

        telemetry.addData("--- Driving ---", "");
//        telemetry.addData("Left Stick Y (Fwd)", "%.2f", -gamepad1.left_stick_y);
//        telemetry.addData("Left Stick X (Str)", "%.2f", gamepad1.left_stick_x);
//        telemetry.addData("Right Stick X (Trn)", "%.2f", -gamepad1.right_stick_x);

        telemetry.addData("--- Actuators ---", "");
        telemetry.addData("Shooting State", shootingState.toString());
        telemetry.addData("Gate Cycle", gateCycleState.toString());
        telemetry.addData("Intake Power", "%.2f", intakePower);
        telemetry.addData("Shooter Power", "%.2f", shooterPower);
        telemetry.addData("Gate A Pos", "%.2f", actuators.getGateAPosition());
        telemetry.addData("Gate B Pos", "%.2f", actuators.getGateBPosition());

        vision.addTelemetry();

        telemetry.update();
    }
}
