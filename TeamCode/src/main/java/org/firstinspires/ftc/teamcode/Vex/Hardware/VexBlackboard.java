package org.firstinspires.ftc.teamcode.Vex.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * A helper class for managing data stored on the FTC SDK blackboard.
 * This centralizes the keys and logic for reading and writing shared data,
 * ensuring consistency between OpModes. This class is designed to be
 * instantiated within an OpMode, to which it holds a reference.
 */
public class VexBlackboard {

    // Key for storing the alliance selection on the blackboard.
    private static final String ALLIANCE_KEY = "Vex::Alliance";
    private static final String STARTING_LOCATION_KEY = "Vex::StartingLocation";
    private static final String POSE_X_KEY = "Vex::PoseX";
    private static final String POSE_Y_KEY = "Vex::PoseY";
    private static final String POSE_HEADING_KEY = "Vex::PoseHeading";


    private OpMode opMode;

    public VexBlackboard(OpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Checks if the Vex-specific blackboard data is empty.
     * @return true if no Vex-specific data is found, false otherwise.
     */
    public boolean isEmpty() {
        return opMode.blackboard.get(ALLIANCE_KEY) == null &&
                opMode.blackboard.get(STARTING_LOCATION_KEY) == null &&
                opMode.blackboard.get(POSE_X_KEY) == null &&
                opMode.blackboard.get(POSE_Y_KEY) == null &&
                opMode.blackboard.get(POSE_HEADING_KEY) == null;
    }

    /**
     * Resets the blackboard values to their defaults.
     */
    public void reset() {
        opMode.blackboard.remove(ALLIANCE_KEY);
        opMode.blackboard.remove(STARTING_LOCATION_KEY);
        opMode.blackboard.remove(POSE_X_KEY);
        opMode.blackboard.remove(POSE_Y_KEY);
        opMode.blackboard.remove(POSE_HEADING_KEY);
    }

    /**
     * Resets the blackboard pose.
     */
    public void resetPose() {
        opMode.blackboard.remove(POSE_X_KEY);
        opMode.blackboard.remove(POSE_Y_KEY);
        opMode.blackboard.remove(POSE_HEADING_KEY);
    }

    /**
     * Retrieves the currently selected alliance from the blackboard.
     *
     * @return The selected Alliance enum. Defaults to UNKNOWN if no selection is found.
     */
    public DecodeField.Alliance getAlliance() {
        String allianceString = (String) opMode.blackboard.getOrDefault(ALLIANCE_KEY, "UNKNOWN");
        try {
            return DecodeField.Alliance.valueOf(allianceString);
        } catch (IllegalArgumentException e) {
            return DecodeField.Alliance.UNKNOWN;
        }
    }

    /**
     * Stores the selected alliance on the blackboard.
     *
     * @param alliance   The Alliance enum to store.
     */
    public void setAlliance(DecodeField.Alliance alliance) {
        opMode.blackboard.put(ALLIANCE_KEY, alliance.toString());
    }

    /**
     * Retrieves the currently selected starting location from the blackboard.
     *
     * @return The selected KeyLocation enum. Defaults to UNKNOWN if no selection is found.
     */
    public DecodeField.KeyLocation getStartingLocation() {
        String locationString = (String) opMode.blackboard.getOrDefault(STARTING_LOCATION_KEY, "UNKNOWN");
        try {
            return DecodeField.KeyLocation.valueOf(locationString);
        } catch (IllegalArgumentException e) {
            return DecodeField.KeyLocation.UNKNOWN;
        }
    }

    /**
     * Stores the selected starting location on the blackboard.
     *
     * @param location   The KeyLocation enum to store.
     */
    public void setStartingLocation(DecodeField.KeyLocation location) {
        if (location != null) {
            opMode.blackboard.put(STARTING_LOCATION_KEY, location.toString());
        } else {
            opMode.blackboard.remove(STARTING_LOCATION_KEY);
        }
    }

    /**
     * Stores the robot's last known pose on the blackboard.
     * @param pose The robot's pose.
     */
    public void setPose(Pose2D pose) {
        opMode.blackboard.put(POSE_X_KEY, pose.getX(DistanceUnit.INCH));
        opMode.blackboard.put(POSE_Y_KEY, pose.getY(DistanceUnit.INCH));
        opMode.blackboard.put(POSE_HEADING_KEY, pose.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Retrieves the robot's last known pose from the blackboard.
     * @return The robot's pose as a Pose3D object, or null if not found.
     */
    public Pose2D getPose() {
        Object xValue = opMode.blackboard.get(POSE_X_KEY);
        Object yValue = opMode.blackboard.get(POSE_Y_KEY);
        Object headingValue = opMode.blackboard.get(POSE_HEADING_KEY);

        if (xValue == null || yValue == null || headingValue == null) {
            return null;
        }

        return new Pose2D(
                DistanceUnit.INCH, (double)xValue, (double)yValue,
                AngleUnit.DEGREES, (double)headingValue
        );
    }

    /**
     * Adds telemetry data about the contents of the Vex blackboard.
     */
    public void addBlackboardTelemetry() {
        opMode.telemetry.addData("--- Blackboard ---", "");
        opMode.telemetry.addData("Alliance", opMode.blackboard.getOrDefault(ALLIANCE_KEY, "UNKNOWN"));
        opMode.telemetry.addData("Start Loc", opMode.blackboard.getOrDefault(STARTING_LOCATION_KEY, "UNKNOWN"));

        Object xValue = opMode.blackboard.get(POSE_X_KEY);
        Object yValue = opMode.blackboard.get(POSE_Y_KEY);
        Object headingValue = opMode.blackboard.get(POSE_HEADING_KEY);

        if (xValue != null && yValue != null && headingValue != null) {
            opMode.telemetry.addData("Stored Pose", "X: %.2f, Y: %.2f, H: %.1f",
                    (double) xValue, (double) yValue, (double) headingValue);
        } else {
            opMode.telemetry.addData("Stored Pose", "Not Set");
        }
    }
}
