package org.firstinspires.ftc.teamcode.Vex.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vex.Hardware.VexBlackboard;

/**
 * Use this OpMode to select the alliance before running the main TeleOp.
 * Press 'X' for BLUE alliance or 'B' for RED alliance.
 * The selection is stored on the blackboard for other OpModes to read.
 */
@TeleOp(name = "[Vex] Game Setup", group = "Vex")
public class VexGameSetup extends OpMode {

    private VexBlackboard blackboardHelper;

    /**
     * This method is called once when the OpMode is initialized.
     */
    @Override
    public void init() {
        blackboardHelper = new VexBlackboard(this);
        telemetry.addData("Alliance Selection", "Press B for RED, X for BLUE");
        updateTelemetry();
    }

    /**
     * This method is called repeatedly during the TeleOp.
     */
    @Override
    public void loop() {
        if (gamepad1.b) {
            blackboardHelper.setAlliance(VexBlackboard.Alliance.RED);
        } else if (gamepad1.x) {
            blackboardHelper.setAlliance(VexBlackboard.Alliance.BLUE);
        }
        updateTelemetry();
    }

    private void updateTelemetry() {
        VexBlackboard.Alliance selection = blackboardHelper.getAlliance();
        telemetry.addData("Alliance Selected", selection.toString());
        telemetry.addData("Instructions", "Press B for RED, X for BLUE.");
        telemetry.addData("Info", "Stop this OpMode after selection and run the main TeleOp.");
        telemetry.update();
    }
}
