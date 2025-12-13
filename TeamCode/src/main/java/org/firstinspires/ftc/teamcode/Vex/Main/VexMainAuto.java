package org.firstinspires.ftc.teamcode.Vex.Main;

import android.util.Log;

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

        // Set initial gate positions for shooting
        actuators.closeGateA();

        if (startingLocation == DecodeField.KeyLocation.UNKNOWN) {
            telemetry.addLine("Cannot execute path, starting location is unknown.");
            telemetry.update();
            return; // Exit if we don't know where we are
        }

        if (startingLocation.name().contains("START_NEAR__FACING_TARGET")) {
            runStartFromNearTargetPath();
        } else if (startingLocation.name().contains("AUDIENCE_WALL")) {
            runAudienceWallPath();
        } else if (startingLocation.name().contains("OBELISK_WALL")) {
            runObeliskWallPath();
        } else {
            telemetry.addData("Warning", "No specific path for " + startingLocation.name());
            telemetry.update();
        }
    }

    // The field is 6x6 tiles (144x144 inches). A tile is 24 inches.

    /**
     * Path for starting from near target.
     * Moves back and shoots.
     */
    private void runStartFromNearTargetPath() {
        telemetry.addLine("Running Start From Target Path...");
        telemetry.update();

        boolean isRed = (currentAlliance == DecodeField.Alliance.RED);
        DecodeField.KeyLocation shootLocation = isRed ?
                DecodeField.KeyLocation.RED__SHOOT_NEAR :
                DecodeField.KeyLocation.BLUE__SHOOT_NEAR;
        double shootX = shootLocation.getPose().getX(DistanceUnit.INCH);
        double shootY = shootLocation.getPose().getY(DistanceUnit.INCH);

        // 1. Drive and shoot
        driveTrain.driveTo(shootX, shootY, 0.4);
        shootCycle_Burst_3();

        // 2. Drive to PPG, and intake
        //   Intake is not in the middle of the robot (shift by 2 inches)
        double targetX = -0.5 * TILE + (isRed ? -2 : 0); // x-offset up
        double targetY = +0.5 * TILE * (isRed ? +1 :-1); // y-mirror

        driveTrain.driveTo(targetX-2, targetY, DRIVE_POWER); // underdrive by 2 inch to prevent overshoot towards +X
        driveTrain.turnToHeading((currentAlliance == DecodeField.Alliance.RED) ? 180 : 0, TURN_POWER);
        actuators.setIntakePower(1);
        driveTrain.driveTo(targetX, 2.36 * TILE * (isRed?1:-1), 0.3);
        // sleep(500); // wait a bit to pick up balls?

        // 3. Drive, turn and shoot
        driveTrain.driveTo(shootX, shootY, DRIVE_POWER);
        turnTowardsCorner(false);
        shootCycle_Burst_3();

        // 4. Turn back to 90 degrees, and drive off the launch line
        driveTrain.turnToHeading(90, TURN_POWER);
        driveTrain.driveRelative(1.5*TILE, 0, 0.4);
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

        // Drive to GPP
        //  Intake is not in the middle of the robot (shift by 2 inches)
        double targetX = +1.5 * TILE + (isRed ? -2 : +2);
        double targetY = +0.5 * TILE * (isRed?1:-1);

        driveTrain.driveTo(targetX+2, targetY, DRIVE_POWER);    // underdrive by 2 inch towards -X
        driveTrain.turnToHeading((currentAlliance == DecodeField.Alliance.RED) ? 180 : 0, TURN_POWER);
        actuators.setIntakePower(1);
        driveTrain.driveTo(targetX, targetY + 2.5*TILE * (isRed?1:-1), 0.4);

        // --- Shooting Sequence ---
        driveToFarTargetAndShoot();

        // 4. Turn back to 90 degrees
        driveTrain.turnToHeading(90, TURN_POWER);

        // 5. Drive off the launch line
        driveTrain.driveRelative(-0.75*TILE, +0.75*TILE * (isRed?1:-1), 0.4);
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

        // Drive to PPG
        //   Intake is not in the middle of the robot (shift by 2 inches)
        double targetX = -0.5 * TILE + (isRed ? -2 : 0);
        double targetY = +0.5 * TILE * (isRed?1:-1);

        driveTrain.driveTo(targetX-2, targetY, DRIVE_POWER); // underdrive by 2 inch to prevent overshoot towards +X
        driveTrain.turnToHeading((currentAlliance == DecodeField.Alliance.RED) ? 180 : 0, TURN_POWER);
        actuators.setIntakePower(1);
        driveTrain.driveTo(targetX, targetY + 2*TILE * (isRed?1:-1), 0.3);

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
        turnTowardsCorner(false);

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
        double targetX = +3.0 * TILE - 9 - 8; // 8 inches from start position to clear wall
        double targetY = +0.5 * TILE * (isRed ? 1 : -1);
        driveTrain.driveTo(targetX, targetY, DRIVE_POWER);

        // 2. Turn to face the alliance corner
        turnTowardsCorner(true);

        // 3. Shoot a predefined number of balls
        shootCycle_Burst_3();
    }

    /**
     * Turns the robot to face the correct alliance corner.
     */
    private void turnTowardsCorner(boolean isFar) {
        boolean isRed = (currentAlliance == DecodeField.Alliance.RED);

        double currentHeading = driveTrain.getHeading();
        double deltaAngle = DecodeField.getTurnAngleToAllianceCorner(currentAlliance, driveTrain.getPose2D());
        double predictedTargetAngle = currentHeading + deltaAngle;

        boolean isAcuteTurn = (Math.abs(deltaAngle) < 90);  // initial turn is acute, second turn is obtuse

        double adjustment;
        double adjustedTargetAngle;

        if (isRed) { // RED
            if (isFar) {    // RED AUDIENCE SIDE
                adjustment = isAcuteTurn ? 0 : 3; // CCW adjustment for acute / obtuse turns -- RED AUDIENCE SIDE
            } else {        // RED OBELISK SIDE
                adjustment = isAcuteTurn ? 0 : 3; // CCW adjustment for acute / obtuse turns -- RED OBELISK SIDE
            }
        } else { // BLUE
            if (isFar) {    // BLUE AUDIENCE SIDE
                adjustment = isAcuteTurn ? 2 : 3; // CCW adjustment for acute / obtuse turns -- BLUE AUDIENCE SIDE
            } else {        // BLUE OBELISK SIDE
                adjustment = isAcuteTurn ? 3 : 6; // CCW adjustment for acute / obtuse turns -- BLUE OBELISK SIDE
            }
        }

        adjustedTargetAngle = predictedTargetAngle + adjustment;

        Log.d(TAG, String.format("turnTowardsCorner: Heading: %.1f, PredictedTarget: %.1f, Delta: %.1f, adjustment: %.1f, adjustedTarget: %.1f",
                currentHeading, predictedTargetAngle, deltaAngle, adjustment, adjustedTargetAngle));

        driveTrain.turnToHeading(adjustedTargetAngle, TURN_POWER);
    }

    private void shootCycle_Burst_3() {
        telemetry.addLine("Starting continuous burst cycle...");
        telemetry.update();

        // Calculate distance to determine logic
        double dist = DecodeField.getDistanceToAllianceCorner(currentAlliance, driveTrain.getPose2D());
        boolean isFar = (dist > 5 * TILE);

        // Set RPM based on distance
        double targetRPM = actuators.predictShooterRPMFromDistance(dist) + (isFar ? 0 : 0);
        actuators.setShooterRPM(targetRPM);

        // Turn on intake to queue the balls
        actuators.setIntakePower(1.0);

        // 6. Wait for shooter RPM to be reached
        spinUpTimer.reset();
        while (opModeIsActive() && spinUpTimer.seconds() < SPIN_UP_TIME_S)
        {
            if (actuators.didShooterReachMinimumTargetRPM(targetRPM)) {
                Log.d(TAG, String.format("TargetRPM %.1f Reached (%.1f) after %.1f seconds",
                        targetRPM, actuators.getShooterRPM(), spinUpTimer.seconds()));
                break;
            }
            actuators.setShooterRPM(targetRPM); // IMPORTANT: Keep RPM updated to do power adjustments
            driveTrain.update(); // IMPORTANT: Keep odometry alive
            idle();
        }

        // Open Gate A to release the stream of balls
        actuators.openGateA();

        // Wait for all 3 balls to shoot
        ElapsedTime burst_timer = new ElapsedTime();
        while (opModeIsActive() && burst_timer.seconds() < BURST_SHOOTING_TIME_S) {
            driveTrain.update();
            actuators.setShooterRPM(targetRPM); // IMPORTANT: Keep RPM updated to do power adjustments
            idle();
        }

        // Turn off
        actuators.setShooterPower(0);
        // actuators.setIntakePower(0);

        // Reset gates and logic for next run
        actuators.closeGateA();
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
    private static final double SPIN_UP_TIME_S = 2.0;
    private static final double BURST_SHOOTING_TIME_S = 3.0;

    private static final double TILE = 24.0; // inches

    private static final String TAG = "VEX::MainAuto";
}
