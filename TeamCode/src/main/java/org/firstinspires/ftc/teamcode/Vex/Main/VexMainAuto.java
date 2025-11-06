package org.firstinspires.ftc.teamcode.Vex.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    private ElapsedTime gateCycleTimer = new ElapsedTime();

    // Constants
    private static final double DRIVE_POWER = 0.5;
    private static final double TURN_POWER = 0.4;
    private static final double SPIN_UP_TIME_S = 1.0;
    private static final long GATE_DELAY_MS = 250;
    private static final int SHOT_COUNT = 3;

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
            telemetry.addLine("Defaulting to (0,0,0). Path may be incorrect.");
            driveTrain.resetPose(0, 0, 0);
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

        // 1. Move forward a little to clear the wall for rotation (e.g., 6 inches)
        // Since the robot starts facing North (positive Y direction), dy is positive.
        driveTrain.driveRelative(0, 6.0, DRIVE_POWER);

        // 2. Turn to face the alliance corner
        turnTowardsCorner();

        // 3. Shoot 3 balls
        shootCycle(SHOT_COUNT);
    }

    /**
     * Path for starting near the obelisk wall.
     * Moves backward to get in shooting range, turns toward the corner, and shoots.
     */
    private void runObeliskWallPath() {
        telemetry.addLine("Running Obelisk Wall Path...");
        telemetry.update();

        // Obelisk is ~3 tiles from corner, too close to shoot.
        // Move back 1.5 tiles (36 inches) to get into a good shooting range.
        // Since the robot starts facing North (positive Y), moving backward means a negative dy.
        driveTrain.driveRelative(0, -36.0, DRIVE_POWER);

        // 2. Turn to face the alliance corner
        turnTowardsCorner();

        // 3. Shoot 3 balls
        shootCycle(SHOT_COUNT);
    }

    /**
     * Turns the robot to face the correct alliance corner.
     */
    private void turnTowardsCorner() {
        double targetAngle = DecodeField.getTurnAngleToAllianceCorner(currentAlliance, driveTrain.getPose());
        driveTrain.turnToHeading(TURN_POWER, targetAngle);
    }

    /**
     * Executes the shooting sequence: spins up the shooter and cycles the gates.
     * @param shots The number of balls to shoot.
     */
    private void shootCycle(int shots) {
        telemetry.addLine("Starting shooting cycle...");
        telemetry.update();

        // 1. Spin up the shooter wheels
        double shooterPower = actuators.calculateDistanceBasedShooterPower(
                DecodeField.getDistanceToAllianceCorner(currentAlliance, driveTrain.getPose())
        );
        actuators.setShooterPower(shooterPower);
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
        Pose3D finalPose = driveTrain.getPose();

        if (finalPose != null) {
            blackboardHelper.setPose(finalPose);
            telemetry.addLine("Final pose saved to blackboard.");
            telemetry.update();
        }
    }
}

