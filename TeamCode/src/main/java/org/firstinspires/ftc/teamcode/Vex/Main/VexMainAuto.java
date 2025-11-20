package org.firstinspires.ftc.teamcode.Vex.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexBlackboard;
import org.firstinspires.ftc.teamcode.Vex.Hardware.DecodeField;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexActuators;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexOdometryDriveTrain;

/**
 * An autonomous OpMode that reads the starting location from the VexBlackboard
 * and executes a shooting routine based on that position. It handles two main
 * scenarios: starting near the Audience Wall or the Obelisk Wall.
 */
@Autonomous(name = "[Vex] Auto Main", group = "Vex")
public class VexMainAuto extends LinearOpMode {

    // Hardware and helper classes
    private VexOdometryDriveTrain driveTrain;
    private VexActuators actuators;
    private VexBlackboard blackboardHelper;

    // State and timers
    private DecodeField.Alliance currentAlliance;
    private DecodeField.KeyLocation startingLocation;
    private ElapsedTime spinUpTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---
        initialize();

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        // --- EXECUTE AUTONOMOUS PATH ---
        executePath();

        // --- SAVE FINAL POSE ---
        saveFinalPose();

        telemetry.addData("Status", "Autonomous Finished!");
        telemetry.update();
        Thread.sleep(1000); // Pause to show final status
    }

    /**
     * Initializes all hardware, reads from the blackboard, and sets the initial pose.
     */
    private void initialize() {
        // Initialize hardware and helper classes
        driveTrain = new VexOdometryDriveTrain(this);
        actuators = new VexActuators(this);
        blackboardHelper = new VexBlackboard(this);

        driveTrain.init();
        actuators.init(hardwareMap);

        // Read alliance and starting location from the blackboard
        currentAlliance = blackboardHelper.getAlliance();
        startingLocation = blackboardHelper.getStartingLocation();

        // Set initial gate positions for shooting
        actuators.closeGateA();
        actuators.openGateB();

        // Set initial pose from blackboard
        if (startingLocation != DecodeField.KeyLocation.UNKNOWN) {
            driveTrain.resetPose(
                    startingLocation.getPose().getX(DistanceUnit.INCH),
                    startingLocation.getPose().getY(DistanceUnit.INCH),
                    startingLocation.getPose().getHeading(AngleUnit.DEGREES)
            );
            telemetry.addData("Status", "Initialized at " + startingLocation.name());
        } else {
            telemetry.addData("ERROR", "Starting location not selected in [Vex] Game Setup!");
            telemetry.addLine("Defaulting to (0,0,90). Path may be incorrect.");
            driveTrain.resetPose(0, 0, 90);
        }
        telemetry.update();
    }

    /**
     * Executes the appropriate autonomous path based on the starting location.
     */
    private void executePath() {
        if (startingLocation == DecodeField.KeyLocation.UNKNOWN) {
            telemetry.addLine("Cannot execute path, starting location is unknown.");
            telemetry.update();
            return; // Exit if we don't know where we are
        }

        // The field is 6x6 tiles (144x144 inches)
        // A tile is 24 inches.
        if (startingLocation.name().contains("AUDIENCE_WALL")) {
            runAudienceWallPath();
        } else if (startingLocation.name().contains("OBELISK_WALL")) {
            runObeliskWallPath();
        } else {
            telemetry.addData("Warning", "No specific path for " + startingLocation.name());
            telemetry.update();
        }
    }

    /**
     * Path for starting near the audience wall.
     * Moves forward to clear the wall, turns toward the corner, and shoots.
     */
    private void runAudienceWallPath() {
        telemetry.addLine("Running Audience Wall Path...");
        telemetry.update();

        boolean isRed = (currentAlliance == DecodeField.Alliance.RED);

        // 1. Move forward a little to clear the wall for rotation (e.g., 6 inches)
        driveToFarTargetAndShoot();

        // 6. Drive to GPP
        double targetX = +1.5 * TILE;
        double targetY = +0.5 * TILE * (isRed?1:-1);

        driveTrain.driveTo(targetX+2, targetY, DRIVE_POWER);    // underdrive by 2 inch
        driveTrain.turnToHeading((currentAlliance == DecodeField.Alliance.RED) ? 180 : 0, TURN_POWER);
        actuators.setIntakePower(1);
        driveTrain.driveTo(targetX, targetY + 2.5*TILE * (isRed?1:-1), 0.4);
        actuators.setIntakePower(1);

        // --- Shooting Sequence ---
        driveToFarTargetAndShoot();

        // 4. Turn back to 90 degrees
        driveTrain.turnToHeading(90, TURN_POWER);

        // 5. Drive off the launch line
        driveTrain.driveRelative(-1.25*TILE, 0, 0.4);
    }

    /**
     * Path for starting near the obelisk wall.
     * Moves backward to get in shooting range, turns toward the corner, and shoots.
     */
    private void runObeliskWallPath() {
        telemetry.addLine("Running Obelisk Wall Path...");
        telemetry.update();

        boolean isRed = (currentAlliance == DecodeField.Alliance.RED);

        // --- Shooting Sequence ---
        driveToNearTargetAndShoot();

        // 6. Drive to PPG
        double targetX = -0.5 * TILE;
        double targetY = +0.5 * TILE * (isRed?1:-1);

        driveTrain.driveTo(targetX-2, targetY, DRIVE_POWER); // underdrive by 2 inch
        driveTrain.turnToHeading((currentAlliance == DecodeField.Alliance.RED) ? 180 : 0, TURN_POWER);
        actuators.setIntakePower(1);
        driveTrain.driveTo(targetX, targetY + 2*TILE * (isRed?1:-1), 0.3);
        actuators.setIntakePower(1);

        // --- Shooting Sequence ---
        driveToNearTargetAndShoot();

        // 4. Turn back to 90 degrees
        driveTrain.turnToHeading(90, TURN_POWER);

        // Move off launch line
        driveTrain.driveRelative(1.5*TILE, 0, 0.4);
    }

    /**
     * Drives the robot to a predefined shooting position, turns toward the corner,
     * and executes a shooting cycle. The target coordinates are hardcoded.
     */
    private void driveToNearTargetAndShoot() {
        // 1. Drive to the shooting position
        boolean isRed = (currentAlliance == DecodeField.Alliance.RED);
        double targetX = -1.0 * TILE;
        double targetY = +0.5 * TILE * (isRed ? 1 : -1);
        driveTrain.driveTo(targetX, targetY, DRIVE_POWER);

        // 2. Turn to face the alliance corner
        turnTowardsCorner(0);

        // 3. Shoot a predefined number of balls
        shootCycle_Burst_3();
    }

    /**
     * Drives the robot to a predefined shooting position, turns toward the corner,
     * and executes a shooting cycle. The target coordinates are hardcoded.
     */
    private void driveToFarTargetAndShoot() {
        // 1. Drive to the shooting position
        boolean isRed = (currentAlliance == DecodeField.Alliance.RED);
        double targetX = +3.0 * TILE - 9 - 6; // 6 inches from start position to clear wall
        double targetY = +0.5 * TILE * (isRed ? 1 : -1);
        driveTrain.driveTo(targetX, targetY, DRIVE_POWER);

        // 2. Turn to face the alliance corner
        turnTowardsCorner(0);

        // 3. Shoot a predefined number of balls
        shootCycle_Burst_3();
    }

    /**
     * Turns the robot to face the correct alliance corner.
     * @param earlyStopByDegrees The number of degrees to reduce the turn by to account for momentum.
     */
    private void turnTowardsCorner(double earlyStopByDegrees) {
        double deltaAngle = DecodeField.getTurnAngleToAllianceCorner(currentAlliance, driveTrain.getPose2D());
        double adjustment = Math.copySign(earlyStopByDegrees, deltaAngle);
        double targetAngle = driveTrain.getHeading() + deltaAngle - adjustment;
        driveTrain.turnToHeading(targetAngle, TURN_POWER);
    }

    private void shootCycle_Burst_3() {
        telemetry.addLine("Starting continuous burst cycle...");
        telemetry.update();

        // Calculate distance to determine logic
        double dist = DecodeField.getDistanceToAllianceCorner(currentAlliance, driveTrain.getPose2D());
        boolean isFar = (dist > 5 * TILE);

        // Set Power Adjustment Factor and enable power adjustment
        actuators.powerAdjustementFactor = (isFar ? 0.30 : 0.15);
        actuators.enablePowerAdjustment = true;

        // Set RPM based on distance
        double targetRPM = actuators.predictShooterRPMFromDistance(dist);
        actuators.setShooterRPM(targetRPM);

        // Turn on intake to pressurize the stack
        actuators.setIntakePower(1.0);

        // 6. Wait for shooter RPM to be reached
        spinUpTimer.reset();
        while (opModeIsActive() && spinUpTimer.seconds() < SPIN_UP_TIME_S)
        {
            if (actuators.isShooterAtTargetRPM(targetRPM)) {
                break;
            }
            driveTrain.update(); // IMPORTANT: Keep odometry alive
            idle();
        }

        // Open Gate A to release the stream of balls
        actuators.openGateA();

        // Wait for all 3 balls to shoot
        sleepWithOdometryUpdate(1500);

        // Turn off
        actuators.setShooterPower(0);
        actuators.setIntakePower(0);

        // Reset gates and logic for next run
        actuators.closeGateA();
        actuators.enablePowerAdjustment = false;
    }

    private void shootCycle_Burst_1_plus_2() {
        telemetry.addLine("Starting shooting cycle...");
        telemetry.update();

        ElapsedTime timerGateA = new ElapsedTime();
        ElapsedTime timerGateB = new ElapsedTime();

        // Calculate distance to determine logic
        double dist = DecodeField.getDistanceToAllianceCorner(currentAlliance, driveTrain.getPose2D());
        boolean isFar = (dist > 5 * TILE);

        // Close Gate B (isolate the first ball)
        actuators.closeGateB();
        timerGateB.reset();

        // Disable power adjustments initially
        actuators.enablePowerAdjustment = false;

        // Set RPM based on distance
        double targetRPM = actuators.predictShooterRPMFromDistance(dist);
        actuators.setShooterRPM(targetRPM);

        // Turn on intake
        actuators.setIntakePower(1.0);

        // Wait for shooter RPM to be reached
        spinUpTimer.reset();
        while (opModeIsActive() && spinUpTimer.seconds() < SPIN_UP_TIME_S)
        {
            if (actuators.isShooterAtTargetRPM(targetRPM) && timerGateB.milliseconds() > GATE_DELAY_MS) {
                break;
            }
            driveTrain.update(); // IMPORTANT: Keep odometry alive
            idle();
        }

        // Set power adjustment factor and enable power adjustment
        actuators.powerAdjustementFactor = (isFar ? 0.4 : 0.3);
        actuators.enablePowerAdjustment = true;

        // Open Gate A for one ball to go
        actuators.openGateA();
        timerGateA.reset();

        // WAIT until at target RPM regained or some timeout
        ElapsedTime recoveryTimer = new ElapsedTime();
        while (opModeIsActive() && recoveryTimer.milliseconds() < 500) {
            if (actuators.isShooterAtTargetRPM(targetRPM) && timerGateA.milliseconds() > GATE_DELAY_MS) {
                break;
            }
            driveTrain.update(); // IMPORTANT: Keep odometry alive
            idle();
        }

        // Open GATE B
        actuators.openGateB();
        timerGateB.reset();

        // Wait some time for 2 more balls to go through
        sleepWithOdometryUpdate(2000);

        // Turn off
        actuators.setShooterPower(0);
        actuators.setIntakePower(0);

        // Reset gates and logic for next run
        actuators.closeGateA();
        actuators.openGateB();
        actuators.enablePowerAdjustment = false;
    }

    /**
     * Executes the shooting sequence: spins up the shooter and cycles the gates.
     */
    private void shootCycle() {
        long shots = SHOT_COUNT;

        telemetry.addLine("Starting shooting cycle...");
        telemetry.update();

        // 1. Spin up the shooter wheels
        actuators.setShooterRPM(
                actuators.predictShooterRPMFromDistance(
                        DecodeField.getDistanceToAllianceCorner(
                                currentAlliance, driveTrain.getPose2D())));

        actuators.setIntakePower(1.0);
        spinUpTimer.reset();
        while (opModeIsActive() && spinUpTimer.seconds() < SPIN_UP_TIME_S) {
            driveTrain.update(); // Keep odometry updated
            telemetry.addData("Shooter", "Spinning up...");
            telemetry.update();
            idle();
        }

        telemetry.addLine("Shooter at speed. Firing...");
        telemetry.update();

        // 2. Cycle the gates to shoot 'shots' number of balls
        for (int i = 0; i < shots && opModeIsActive(); i++) {
            // STEP 1: Close Gate B to isolate one ball
            actuators.closeGateB();
            sleepWithOdometryUpdate(GATE_DELAY_MS);

            // STEP 2: Open Gate A to let the ball into the shooter
            actuators.openGateA();
            sleepWithOdometryUpdate(GATE_DELAY_MS);

            // STEP 3: Close Gate A to prepare for the next ball
            actuators.closeGateA();
            sleepWithOdometryUpdate(GATE_DELAY_MS);

            // STEP 4: Open Gate B to let the next ball into the chamber
            actuators.openGateB();
            sleepWithOdometryUpdate(GATE_DELAY_MS);

            telemetry.addData("Shot", (i + 1) + " of " + shots);
            telemetry.update();
        }

        // 3. Stop the shooter motor
        actuators.setShooterPower(0);
    }

    /**
     * A replacement for sleep() that continuously updates odometry.
     * @param milliseconds The duration to wait.
     */
    private void sleepWithOdometryUpdate(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            driveTrain.update();
            idle();
        }
    }

    /**
     * Saves the robot's final pose to the blackboard for TeleOp to use.
     */
    private void saveFinalPose() {
        driveTrain.stopMotors();
        driveTrain.update(); // Final odometry update
        Pose2D finalPose = driveTrain.getPose2D();

        if (finalPose != null) {
            blackboardHelper.setPose(finalPose);
            telemetry.addLine("Final pose saved to blackboard.");
            telemetry.update();
        }
    }

    // Constants
    private static final double DRIVE_POWER = 0.8;
    private static final double TURN_POWER = 0.5;
    private static final double SPIN_UP_TIME_S = 1.25;
    private static final long GATE_DELAY_MS = 350;
    private static final long SHOT_COUNT = 4;
    private static final double TILE = 24.0; // inches
}

