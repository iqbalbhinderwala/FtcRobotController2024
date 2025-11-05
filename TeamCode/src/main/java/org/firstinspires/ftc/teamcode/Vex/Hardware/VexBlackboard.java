package org.firstinspires.ftc.teamcode.Vex.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A helper class for managing data stored on the FTC SDK blackboard.
 * This centralizes the keys and logic for reading and writing shared data,
 * ensuring consistency between OpModes. This class is designed to be
 * instantiated within an OpMode, to which it holds a reference.
 */
public class VexBlackboard {

    // Enum for Alliance selection
    public enum Alliance { RED, BLUE, UNKNOWN }

    // Key for storing the alliance selection on the blackboard.
    private static final String ALLIANCE_KEY = "Vex::Alliance";

    private OpMode opMode;

    public VexBlackboard(OpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Retrieves the currently selected alliance from the blackboard.
     *
     * @return The selected Alliance enum. Defaults to UNKNOWN if no selection is found.
     */
    public Alliance getAlliance() {
        String allianceString = (String) opMode.blackboard.getOrDefault(ALLIANCE_KEY, "UNKNOWN");
        try {
            return Alliance.valueOf(allianceString);
        } catch (IllegalArgumentException e) {
            return Alliance.UNKNOWN;
        }
    }

    /**
     * Stores the selected alliance on the blackboard.
     *
     * @param alliance   The Alliance enum to store.
     */
    public void setAlliance(Alliance alliance) {
        opMode.blackboard.put(ALLIANCE_KEY, alliance.toString());
    }
}
