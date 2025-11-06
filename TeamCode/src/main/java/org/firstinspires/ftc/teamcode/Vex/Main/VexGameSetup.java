package org.firstinspires.ftc.teamcode.Vex.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Vex.Hardware.DecodeField;
import org.firstinspires.ftc.teamcode.Vex.Hardware.VexBlackboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Use this OpMode to select the starting position using a two-step menu.
 * 1. Press 'B' for RED alliance or 'X' for BLUE alliance.
 * 2. Use the D-Pad (Up/Down) to select the specific start location.
 * 3. Press 'A' to confirm your selection.
 * The selection is stored on the blackboard for other OpModes to read.
 */
@TeleOp(name = "[Vex] Game Setup", group = "Vex")
public class VexGameSetup extends LinearOpMode {

    private enum SetupState {
        SELECT_ALLIANCE,
        SELECT_LOCATION,
        CONFIRMED
    }

    @Override
    public void runOpMode() throws InterruptedException {
        VexBlackboard blackboardHelper = new VexBlackboard(this);
        blackboardHelper.reset(); // Always start fresh

        // --- State Variables ---
        SetupState currentState = SetupState.SELECT_ALLIANCE;
        List<DecodeField.KeyLocation> filteredLocations = new ArrayList<>();
        int selectionIndex = 0;
        boolean lastDpadUp = false, lastDpadDown = false, lastA = false, lastY = false, lastB = false, lastX = false;

        // --- OpMode Loop (runs during INIT) ---
        while (!isStarted() && !isStopRequested()) {
            telemetry.clear();

            switch (currentState) {
                case SELECT_ALLIANCE:
                    telemetry.addLine("Select Alliance:");
                    telemetry.addLine("Press [B] for RED");
                    telemetry.addLine("Press [X] for BLUE");

                    if (gamepad1.b && !lastB) {
                        populateFilteredLocations("RED", filteredLocations);
                        currentState = SetupState.SELECT_LOCATION;
                    } else if (gamepad1.x && !lastX) {
                        populateFilteredLocations("BLUE", filteredLocations);
                        currentState = SetupState.SELECT_LOCATION;
                    }
                    break;

                case SELECT_LOCATION:
                    telemetry.addLine("Select Starting Position:");
                    telemetry.addLine("(D-Pad Up/Down, Press A to confirm)");
                    telemetry.addLine();

                    for (int i = 0; i < filteredLocations.size(); i++) {
                        String prefix = (i == selectionIndex) ? " > " : "   ";
                        telemetry.addLine(String.format(Locale.US, "%s%s", prefix, filteredLocations.get(i).toString()));
                    }

                    boolean dpadUp = gamepad1.dpad_up;
                    if (dpadUp && !lastDpadUp) {
                        selectionIndex = (selectionIndex - 1 + filteredLocations.size()) % filteredLocations.size();
                    }
                    boolean dpadDown = gamepad1.dpad_down;
                    if (dpadDown && !lastDpadDown) {
                        selectionIndex = (selectionIndex + 1) % filteredLocations.size();
                    }

                    if (gamepad1.a && !lastA) {
                        saveSelectionToBlackboard(blackboardHelper, filteredLocations.get(selectionIndex));
                        currentState = SetupState.CONFIRMED;
                    } else if (gamepad1.y && !lastY) { // Allow reset/back
                        currentState = SetupState.SELECT_ALLIANCE;
                        blackboardHelper.reset();
                    }
                    lastDpadUp = dpadUp;
                    lastDpadDown = dpadDown;
                    break;

                case CONFIRMED:
                    telemetry.addLine("✓ Selection Saved");
                    telemetry.addLine();
                    telemetry.addData("Alliance", blackboardHelper.getAlliance().toString());
                    telemetry.addData("Starting Location", blackboardHelper.getStartingLocation().toString());
                    telemetry.addLine();
                    telemetry.addLine("Press START to finish, or Y to reset.");

                    if (gamepad1.y && !lastY) {
                        currentState = SetupState.SELECT_ALLIANCE;
                        blackboardHelper.reset();
                        selectionIndex = 0;
                    }
                    break;
            }

            // Update last gamepad states
            lastA = gamepad1.a;
            lastY = gamepad1.y;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            telemetry.update();
            idle();
        }

        waitForStart();
        // The rest of your OpMode can run here, or just end to lock in the selection.
    }

    private void populateFilteredLocations(String allianceColor, List<DecodeField.KeyLocation> filteredList) {
        filteredList.clear();
        for (DecodeField.KeyLocation loc : DecodeField.KeyLocation.values()) {
            if (loc.name().startsWith(allianceColor)) {
                filteredList.add(loc);
            }
        }
    }

    private void saveSelectionToBlackboard(VexBlackboard blackboard, DecodeField.KeyLocation selection) {
        blackboard.setStartingLocation(selection);
        if (selection.name().startsWith("RED")) {
            blackboard.setAlliance(DecodeField.Alliance.RED);
        } else if (selection.name().startsWith("BLUE")) {
            blackboard.setAlliance(DecodeField.Alliance.BLUE);
        }
    }
}
