package org.firstinspires.ftc.teamcode.John.Hardware;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;

/**
 * Dual intake STUB: no hardware yet. It only tracks which intake is running and reports it.
 * Replace the bodies of on / off with the real motors later; the autonomous keeps using the
 * commands.
 *
 * John has an intake at each axial end, collecting toward the middle (RobotGeometry): intake A on
 * the shooter's left (Pedro's back end), intake B on its right (Pedro's front end).
 */
@Config("JohnIntake")
public class Intake {

    public enum Side { A, B }

    /** Time to keep running after the robot stops, so the last ball gets pulled in. */
    public static double SETTLE_MS = 300;

    private Side running = null;

    public void on(Side side) {
        running = side;
    }

    public void off() {
        running = null;
    }

    public String status() {
        return (running == null ? "off" : "intake " + running + " running") + " (stub)";
    }

    public Command start(Side side) {
        return instant(() -> on(side));
    }

    public Command stop() {
        return instant(this::off);
    }
}
